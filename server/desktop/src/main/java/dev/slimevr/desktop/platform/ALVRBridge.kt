package dev.slimevr.desktop.platform

import dev.slimevr.VRServer
import dev.slimevr.bridge.Bridge
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import io.eiren.util.logging.LogManager
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Sends SlimeVR-computed tracker poses to ALVR via UDP IPC.
 *
 * ALVR listens on 127.0.0.1:21298 and injects any received device motions
 * (for devices not already reported by the VR headset) into its tracking
 * pipeline.
 *
 * # Packet layout (little-endian)
 *
 * Header (16 bytes):
 *   magic[4]          = 0x53 0x4C 0x56 0x52  ("SLVR")
 *   version[1]        = 1
 *   tracker_count[1]  = number of per-tracker records
 *   _pad[2]           = 0x00 0x00
 *   timestamp_ns[8]   = System.nanoTime()
 *
 * Per-tracker record (56 bytes):
 *   tracker_type[1]   = see TRACKER_* companion constants
 *   _pad[3]           = 0x00 × 3
 *   px,py,pz[3×f32]   = world-space position (metres)
 *   qx,qy,qz,qw[4×f32]= world-space orientation quaternion
 *   vx,vy,vz[3×f32]   = linear velocity m/s  (zero when unavailable)
 *   ax,ay,az[3×f32]   = angular velocity rad/s (zero when unavailable)
 */
class ALVRBridge(
	@Suppress("UNUSED_PARAMETER") server: VRServer,
) : Bridge {

	companion object {
		const val TAG = "ALVRBridge"
		const val ALVR_BRIDGE_PORT = 21298

		private val MAGIC = byteArrayOf(0x53, 0x4C, 0x56.toByte(), 0x52)
		private const val PROTOCOL_VERSION: Byte = 1
		private const val HEADER_SIZE = 16
		private const val RECORD_SIZE = 56

		// Tracker-type IDs — must stay in sync with slimevr_bridge.rs
		const val TRACKER_HEAD: Byte = 0
		const val TRACKER_CHEST: Byte = 1
		const val TRACKER_HIPS: Byte = 2
		const val TRACKER_LEFT_ELBOW: Byte = 3
		const val TRACKER_RIGHT_ELBOW: Byte = 4
		const val TRACKER_LEFT_KNEE: Byte = 5
		const val TRACKER_RIGHT_KNEE: Byte = 6
		const val TRACKER_LEFT_FOOT: Byte = 7
		const val TRACKER_RIGHT_FOOT: Byte = 8

		private val POSITION_TO_TYPE: Map<TrackerPosition, Byte> = mapOf(
			TrackerPosition.HEAD to TRACKER_HEAD,
			TrackerPosition.UPPER_CHEST to TRACKER_CHEST,
			TrackerPosition.HIP to TRACKER_HIPS,
			TrackerPosition.LEFT_UPPER_ARM to TRACKER_LEFT_ELBOW,
			TrackerPosition.RIGHT_UPPER_ARM to TRACKER_RIGHT_ELBOW,
			TrackerPosition.LEFT_UPPER_LEG to TRACKER_LEFT_KNEE,
			TrackerPosition.RIGHT_UPPER_LEG to TRACKER_RIGHT_KNEE,
			TrackerPosition.LEFT_FOOT to TRACKER_LEFT_FOOT,
			TrackerPosition.RIGHT_FOOT to TRACKER_RIGHT_FOOT,
		)
	}

	private var socket: DatagramSocket? = null
	private val destAddress = InetSocketAddress("127.0.0.1", ALVR_BRIDGE_PORT)

	/** Trackers that will be sent to ALVR, paired with their type byte. */
	private val sharedTrackers = mutableListOf<Pair<Byte, Tracker>>()

	// ── Bridge lifecycle ──────────────────────────────────────────────────────

	override fun startBridge() {
		try {
			socket = DatagramSocket()
			LogManager.info("[$TAG] Started — sending tracker data to 127.0.0.1:$ALVR_BRIDGE_PORT")
		} catch (e: Exception) {
			LogManager.warning("[$TAG] Failed to open UDP socket: ${e.message}")
		}
	}

	override fun isConnected(): Boolean = socket != null

	// ── Received data direction (ALVR → SlimeVR) — not used ──────────────────

	override fun dataRead() {
		// This bridge is send-only; ALVR does not push data back via this port.
	}

	// ── Send computed tracker poses to ALVR ──────────────────────────────────

	override fun dataWrite() {
		val sock = socket ?: return
		val snapshot = synchronized(sharedTrackers) { sharedTrackers.toList() }
		if (snapshot.isEmpty()) return

		val count = snapshot.size
		val buf = ByteBuffer
			.allocate(HEADER_SIZE + count * RECORD_SIZE)
			.order(ByteOrder.LITTLE_ENDIAN)

		// Header
		buf.put(MAGIC)
		buf.put(PROTOCOL_VERSION)
		buf.put(count.toByte())
		buf.put(0) // padding
		buf.put(0) // padding
		buf.putLong(System.nanoTime())

		// Per-tracker records
		for ((typeId, tracker) in snapshot) {
			val pos = tracker.position
			val rot = tracker.getRotation()

			buf.put(typeId)
			buf.put(0) // padding
			buf.put(0) // padding
			buf.put(0) // padding

			// Position (x, y, z)
			buf.putFloat(pos.x)
			buf.putFloat(pos.y)
			buf.putFloat(pos.z)

			// Orientation (x, y, z, w) — ktmath Quaternion field order
			buf.putFloat(rot.x)
			buf.putFloat(rot.y)
			buf.putFloat(rot.z)
			buf.putFloat(rot.w)

			// Velocity — SlimeVR computed trackers do not expose velocity,
			// so we send zero.  ALVR will still function correctly; it just
			// won't be able to extrapolate beyond the last received pose.
			buf.putFloat(0f) // vx
			buf.putFloat(0f) // vy
			buf.putFloat(0f) // vz
			buf.putFloat(0f) // ax
			buf.putFloat(0f) // ay
			buf.putFloat(0f) // az
		}

		try {
			val bytes = buf.array()
			sock.send(DatagramPacket(bytes, bytes.size, destAddress))
		} catch (e: Exception) {
			LogManager.warning("[$TAG] Send failed: ${e.message}")
		}
	}

	// ── Tracker registration ──────────────────────────────────────────────────

	override fun addSharedTracker(tracker: Tracker?) {
		if (tracker == null) return
		val position = tracker.trackerPosition ?: return
		val typeId = POSITION_TO_TYPE[position] ?: return
		synchronized(sharedTrackers) {
			if (sharedTrackers.none { it.second === tracker }) {
				sharedTrackers.add(typeId to tracker)
				LogManager.info("[$TAG] Sharing tracker: ${tracker.name} (${position.name})")
			}
		}
	}

	override fun removeSharedTracker(tracker: Tracker?) {
		if (tracker == null) return
		synchronized(sharedTrackers) {
			sharedTrackers.removeIf { it.second === tracker }
		}
	}
}
