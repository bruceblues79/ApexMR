package dev.apexmr.core.input

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

data class BikeFeedback(
    val body: Pose,
    val handles: Pose,
    val handleScaleX: Double = 1.0
) {
    companion object {
        fun solve(base: CalibrationFrame?, frame: HandSnapshot?, intent: RiderIntent, config: InputConfig): BikeFeedback {
            val nominal = Vec3(0.0, config.handleHeightM, -config.handleForwardM)
            val fallback = BikeFeedback(Pose(rotation = rollRotation(intent.lean * config.leanFullRad)), Pose(nominal))
            if (base == null || frame == null || intent.status != InputStatus.DRIVING) return fallback
            if (!intent.leftGrip || !intent.rightGrip) return fallback
            if (frame.left?.valid() != true || frame.right?.valid() != true) return fallback
            if (!frame.timestampS.isFinite() || !frame.sampleTimestampS.isFinite() ||
                frame.timestampS - frame.sampleTimestampS !in 0.0..config.inputTimeoutS) return fallback
            val rig = if (frame.coordinateSpace == "rig") RigFrame() else base.rig
            val left = rig.toLocal(frame.left.palm.position)
            val right = rig.toLocal(frame.right.palm.position)
            val line = right - left
            if (line.length() <= MIN_SPAN_M) return fallback
            val baseRig = base.coordinateRig()
            val left0 = baseRig.toLocal(base.snapshot.left!!.palm.position)
            val right0 = baseRig.toLocal(base.snapshot.right!!.palm.position)
            val line0 = right0 - left0
            val yaw0 = atan2(line0.z, line0.x)
            val roll0 = -atan2(line0.y, hypot(line0.x, line0.z))
            val yawDelta = atan2(sin(atan2(line.z, line.x) - yaw0), cos(atan2(line.z, line.x) - yaw0))
            val rollDelta = (-atan2(line.y, hypot(line.x, line.z)) - roll0)
                .coerceIn(-config.handleRollLimitRad, config.handleRollLimitRad)
            val handleRotation = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0),
                -yawDelta.coerceIn(-config.handleYawLimitRad, config.handleYawLimitRad))
            return BikeFeedback(
                Pose(rotation = rollRotation(rollDelta)),
                Pose(nominal, handleRotation)
            )
        }

        private fun rollRotation(angle: Double) = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), angle)
        private const val MIN_SPAN_M = 0.10
    }
}
