package dev.apexmr.core.input

import kotlin.math.*

internal data class GestureIntent(val throttle: Double, val brake: Double, val brakeReady: Boolean, val armed: Boolean)

internal class RightGestures(private val config: InputConfig) {
    private var armed = false
    private var brakeReady = false
    private var dwellS = 0.0
    private var referenceCurl = 0.0
    private var referenceTip = Vec3()

    fun reset() { armed = false; brakeReady = false; dwellS = 0.0 }
    fun update(right: HandPose?, base: CalibrationFrame, handle: HandlePose, usable: Boolean, dt: Double): GestureIntent {
        if (right == null || !usable) { reset(); return GestureIntent(0.0, 0.0, false, false) }
        val fist = right.curls.all { it >= config.fistCurl }
        val extended = right.curls.take(2).all { it <= config.extendedCurl } &&
            right.curls.drop(2).all { it >= config.fistCurl }
        val tips = right.fingerTips.map {
            handle.rotation.inverse().rotate(base.coordinateRig().toLocal(it) - handle.midpoint0) + handle.midpoint0
        }
        val tip = (tips[0] + tips[1]) * 0.5
        val curl = (right.curls[0] + right.curls[1]) * 0.5
        updateBrake(extended, fist, right.curls.drop(2).all { it >= config.fistCurl }, tip, curl, handle, dt)
        val preparing = extended || brakeReady
        val twist = twist(right, base, handle)
        updateArmed(fist, preparing, twist)
        val throttle = if (armed) normalized(twist, config.throttleDeadRad, config.throttleFullRad).coerceAtLeast(0.0) else 0.0
        val curlPull = (curl - referenceCurl) / config.brakeCurlRange
        val travelPull = (tip.z - referenceTip.z) / config.brakeTravelM
        val brake = if (brakeReady) min(curlPull, travelPull).coerceIn(0.0, 1.0) else 0.0
        return GestureIntent(throttle, brake, brakeReady, armed)
    }
    private fun updateBrake(extended: Boolean, fist: Boolean, supportingFingers: Boolean,
                            tip: Vec3, curl: Double, handle: HandlePose, dt: Double) {
        val lever = handle.right0 + Vec3(0.0, 0.0, -config.brakeOffsetM)
        val contact = extended && (tip - lever).length() <= config.brakeContactM
        dwellS = if (contact) dwellS + dt else 0.0
        if (!brakeReady && dwellS >= config.brakeDwellS) {
            brakeReady = true
            referenceCurl = curl
            referenceTip = tip
        }
        if (fist || !supportingFingers) brakeReady = false
    }
    private fun updateArmed(fist: Boolean, preparing: Boolean, twist: Double) {
        if (!fist || preparing) armed = false
        val atZero = abs(twist) <= config.throttleDeadRad
        if (fist && !preparing && atZero) armed = true
    }
    private fun twist(right: HandPose, base: CalibrationFrame, handle: HandlePose): Double {
        val current = base.coordinateRig().rotation().inverse() * right.palm.rotation
        val zero = base.coordinateRig().rotation().inverse() * base.snapshot.right!!.palm.rotation
        val relative = handle.rotation.inverse() * current * zero.inverse()
        var angle = 2 * atan2(relative.x, relative.w)
        if (angle > PI) angle -= 2 * PI
        if (angle < -PI) angle += 2 * PI
        return angle
    }
}
