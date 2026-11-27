package dev.apexmr.prototype.input

import android.os.SystemClock
import android.content.Context
import android.content.pm.PackageManager
import androidx.core.content.ContextCompat
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.core.math.Quat
import com.pico.spatial.tracking.DataProvider
import com.pico.spatial.tracking.hand.*
import com.pico.spatial.tracking.hmd.*
import dev.apexmr.core.input.*
import dev.apexmr.core.input.HandPose as CoreHandPose
import com.pico.spatial.tracking.hand.HandPose as SdkHandPose

class TrackingAdapter(private val root: Entity, private val context: Context) {
    val hands = HandTrackingProvider()
    private val head = HMDTrackingProvider()
    private var started = false
    private data class Received<T>(val data: T, val timeS: Double)
    @Volatile private var handSample: Received<HandTrackingData>? = null
    @Volatile private var headSample: Received<HMDTrackingData>? = null
    private val handListener = DataProvider.DataListener<HandTrackingData> {
        handSample = Received(it, now())
    }
    private val headListener = DataProvider.DataListener<HMDTrackingData> {
        headSample = Received(it, now())
    }

    fun start() {
        if (!started) {
            started = true
            hands.addListener(handListener); head.addListener(headListener)
            head.start()
        }
        if (ContextCompat.checkSelfPermission(context, "com.picovr.permission.HAND_TRACKING") ==
            PackageManager.PERMISSION_GRANTED && hands.state != DataProvider.State.STARTED) hands.start()
    }
    fun stop() {
        if (!started) return
        started = false
        hands.removeListener(handListener); head.removeListener(headListener)
        hands.stop(); head.stop()
        handSample = null; headSample = null
    }
    fun snapshot(): HandSnapshot {
        val now = now()
        val sample = handSample
        val data = sample?.data
        val rawLeft = data?.left?.let { convertHand(it, data.leftPinch, data.leftAim, false) }
        val rawRight = data?.right?.let { convertHand(it, data.rightPinch, data.rightAim, false) }
        val h = headSample?.data?.hmdPose
        val rawHmd = h?.let { pose(it.position, it.rotation, false) }
        val hmdFresh = headSample?.let { now - it.timeS <= 0.15 } == true
        val hmd = if (h != null && hmdFresh) pose(h.position, h.rotation, true)
            else Pose(rotation = Rotation(0.0, 0.0, 0.0, 0.0))
        return HandSnapshot(
            now, sample?.timeS ?: now - 1.0,
            data?.left?.let { convertHand(it, data.leftPinch, data.leftAim, true) },
            data?.right?.let { convertHand(it, data.rightPinch, data.rightAim, true) },
            hmd, rawLeft, rawRight, rawHmd,
            pose(Vector3.ZERO, Quat.identity(), true)
        )
    }

    private fun convertHand(hand: SdkHandPose, pinch: HandPinch?, aim: HandAim?, local: Boolean): CoreHandPose? {
        val joints = HandJoint.Index.entries.map { index -> hand[index].let { pose(it.position, it.rotation, local) } }
        val curls = listOf(6 to 8, 11 to 13, 16 to 18, 21 to 23).map { (base, bend) ->
            HandGeometry.curl(joints[base + 1].position, joints[bend].position, joints[base + 4].position)
                ?: return null
        }
        return CoreHandPose(
            joints[0], curls, listOf(joints[10].position, joints[15].position), joints,
            pinch?.pinchValue?.toDouble(), aim?.aimPose?.let { pose(it.position, it.rotation, local) },
            pinch?.pinchPose?.let { pose(it.position, it.rotation, local) }, pinch?.isReady == true,
            aim?.aimValue?.toDouble(), aim?.isReady == true
        )
    }
    private fun pose(position: Vector3, rotation: Quat, local: Boolean): Pose {
        val p = if (local) root.convertPositionFrom(position, null) else position
        val q = if (local) root.convertRotationFrom(rotation, null) else rotation
        return Pose(Vec3(p.x.toDouble(), p.y.toDouble(), p.z.toDouble()),
            Rotation(q.x.toDouble(), q.y.toDouble(), q.z.toDouble(), q.w.toDouble()))
    }
    private fun now() = SystemClock.elapsedRealtimeNanos() / 1e9
}
