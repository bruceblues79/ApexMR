package dev.apexmr.core.input

import kotlin.math.*

internal class SmoothSignal {
    private var value = 0.0
    private var previous = 0.0
    fun step(target: Double, dt: Double, config: InputConfig): Double {
        val speed = abs(target - previous) / dt
        previous = target
        val cutoff = config.filterCutoffHz + config.filterBeta * speed
        val alpha = 1.0 / (1.0 + 1.0 / (2 * PI * cutoff * dt))
        value += alpha * (target - value)
        return value
    }
}

internal class GripLatch {
    var active = false
    private var elapsed = 0.0
    fun update(distance: Double, dt: Double, config: InputConfig): Boolean {
        val condition = if (active) distance > config.gripExitM else distance <= config.gripEnterM
        elapsed = if (condition) elapsed + dt else 0.0
        if (elapsed >= if (active) config.gripExitS else config.gripEnterS) {
            active = !active
            elapsed = 0.0
        }
        return active
    }
}

internal fun normalized(v: Double, dead: Double, full: Double) =
    if (abs(v) <= dead) 0.0 else sign(v) * ((abs(v) - dead) / (full - dead)).coerceAtMost(1.0)

internal data class HandlePose(
    val yaw: Double, val roll: Double, val body: Double,
    val leftDistance: Double, val rightDistance: Double,
    val rotation: Rotation, val midpoint0: Vec3, val right0: Vec3
)

internal fun CalibrationFrame.coordinateRig() = if (snapshot.coordinateSpace == "rig") RigFrame() else rig

internal fun handlePose(base: CalibrationFrame, frame: HandSnapshot, left: HandPose?, right: HandPose?,
                        config: InputConfig): HandlePose {
    val rig = base.coordinateRig()
    val l0 = rig.toLocal(base.snapshot.left!!.palm.position)
    val r0 = rig.toLocal(base.snapshot.right!!.palm.position)
    val midpoint0 = (l0 + r0) * 0.5
    val line0 = r0 - l0
    val l = left?.let { rig.toLocal(it.palm.position) }
    val r = right?.let { rig.toLocal(it.palm.position) }
    val line = if (l != null && r != null) r - l else line0
    val midpoint = if (l != null && r != null) (l + r) * 0.5 else midpoint0
    val yaw = atan2(line.z, line.x) - atan2(line0.z, line0.x)
    val roll = -atan2(line.y, hypot(line.x, line.z)) + atan2(line0.y, hypot(line0.x, line0.z))
    val rotation = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -yaw) *
        Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), roll)
    val permitted = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -yaw.coerceIn(-config.handleYawLimitRad, config.handleYawLimitRad)) *
        Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), roll.coerceIn(-config.handleRollLimitRad, config.handleRollLimitRad))
    val expectedL = midpoint0 + permitted.rotate(l0 - midpoint0)
    val expectedR = midpoint0 + permitted.rotate(r0 - midpoint0)
    val limitError = max(0.0, (midpoint - midpoint0).length() - config.midpointLimitM) +
        max(0.0, abs(line.length() - line0.length()) - config.spacingToleranceM)
    val ld = l?.let { (it - expectedL).length() + limitError } ?: Double.POSITIVE_INFINITY
    val rd = r?.let { (it - expectedR).length() + limitError } ?: Double.POSITIVE_INFINITY
    val head0 = rig.toLocal(base.snapshot.hmd.position)
    val head = if (frame.hmd.valid()) rig.toLocal(frame.hmd.position) else head0
    val extraBodyX = (head.x - head0.x) - (midpoint.x - midpoint0.x) - sin(roll) * head0.y
    return HandlePose(yaw, roll, normalized(extraBodyX, config.bodyDeadM, config.bodyFullM), ld, rd,
        rotation, midpoint0, r0)
}

class InputEngine(val config: InputConfig) {
    var calibration: CalibrationFrame? = null
        private set
    private var lastTime = Double.NEGATIVE_INFINITY
    private var paused = false
    private var leftGrip = GripLatch()
    private var rightGrip = GripLatch()
    private var filters = List(5) { SmoothSignal() }
    private var gestures = RightGestures(config)
    private var bothLostSinceS: Double? = null
    private var result = RiderIntent()
    var rejectedFrames = 0L
        private set

    init { config.validate() }

    fun calibrate(snapshot: HandSnapshot, rig: RigFrame) {
        require(snapshot.left?.valid() == true && snapshot.right?.valid() == true && snapshot.hmd.valid())
        require(snapshot.timestampS.isFinite() && snapshot.sampleTimestampS.isFinite())
        require(rig.origin.valid() && rig.yawRad.isFinite())
        val spacing = (snapshot.right!!.palm.position - snapshot.left!!.palm.position).length()
        require(spacing > 0.10 && spacing < 1.0)
        calibration = CalibrationFrame(snapshot, rig)
        lastTime = snapshot.timestampS
        leftGrip = GripLatch()
        rightGrip = GripLatch()
        filters = List(5) { SmoothSignal() }
        gestures = RightGestures(config)
        paused = false
        bothLostSinceS = null
        result = RiderIntent()
    }
    fun pause() { paused = true; gestures.reset(); filters = List(5) { SmoothSignal() } }
    fun resume() { paused = false; gestures.reset() }

    fun process(frame: HandSnapshot): RiderIntent {
        if (!frame.timestampS.isFinite() || frame.timestampS <= lastTime) {
            rejectedFrames++
            return result
        }
        val elapsedDt = if (lastTime.isFinite()) frame.timestampS - lastTime else FIRST_INPUT_DT_S
        val dt = elapsedDt.coerceAtMost(MAX_INPUT_DT_S)
        lastTime = frame.timestampS
        val base = calibration ?: return result
        if (paused) return pausedResult()
        val fresh = isFresh(frame)
        val left = frame.left?.takeIf { fresh && it.valid() }
        val right = frame.right?.takeIf { fresh && it.valid() }
        bothLostSinceS = if (left == null && right == null) bothLostSinceS ?: frame.timestampS else null
        val lostDuration = bothLostSinceS?.let { frame.timestampS - it } ?: 0.0
        if (lostDuration >= config.bothLostPauseS) { pause(); return pausedResult() }
        return updateIntent(base, frame, left, right, dt)
    }

    private fun isFresh(frame: HandSnapshot) = frame.sampleTimestampS.isFinite() &&
        frame.timestampS - frame.sampleTimestampS in 0.0..config.inputTimeoutS && frame.hmd.valid()

    private fun pausedResult(): RiderIntent {
        result = RiderIntent(status = InputStatus.PAUSED)
        return result
    }

    private fun updateIntent(base: CalibrationFrame, frame: HandSnapshot, left: HandPose?, right: HandPose?,
                             dt: Double): RiderIntent {
        val pose = handlePose(base, frame, left, right, config)
        val lg = leftGrip.update(pose.leftDistance, dt, config) && left != null
        val rg = rightGrip.update(pose.rightDistance, dt, config) && right != null
        val tracked = left != null && right != null
        val control = lg && rg && tracked
        val steer = if (control) normalized(pose.yaw, config.steerDeadRad, config.steerFullRad) else result.steer * decay(dt)
        val lean = if (control) normalized(pose.roll, config.leanDeadRad, config.leanFullRad) else result.lean * decay(dt)
        val body = if (control) pose.body else 0.0
        val gesture = gestures.update(right, base, pose, rg && tracked, dt)
        val throttle = filters[0].step(gesture.throttle, dt, config)
        val brake = filters[1].step(gesture.brake, dt, config)
        result = RiderIntent(
            if (gesture.throttle == 0.0) 0.0 else throttle,
            if (!gesture.brakeReady) 0.0 else brake,
            filters[2].step(steer, dt, config), filters[3].step(lean, dt, config),
            filters[4].step(body, dt, config), lg, rg,
            (pose.leftDistance / config.gripExitM).coerceIn(0.0, 1.0),
            (pose.rightDistance / config.gripExitM).coerceIn(0.0, 1.0),
            left != null, right != null, gesture.brakeReady, gesture.armed, status(tracked, control)
        )
        return result
    }
    private fun status(tracked: Boolean, control: Boolean) = when {
        !tracked -> InputStatus.TRACKING_LOST
        control -> InputStatus.DRIVING
        else -> InputStatus.RELEASED
    }
    private fun decay(dt: Double) = (1.0 - dt / config.lossDecayS).coerceAtLeast(0.0)
    private companion object {
        const val MAX_INPUT_DT_S = 0.10
        const val FIRST_INPUT_DT_S = 0.02
    }
}
