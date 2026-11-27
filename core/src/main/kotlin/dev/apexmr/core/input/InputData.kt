package dev.apexmr.core.input

import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

val inputJson = Json { encodeDefaults = true }

@Serializable
data class InputConfig(
    val gripEnterM: Double = 0.10,
    val gripExitM: Double = 0.16,
    val gripEnterS: Double = 0.15,
    val gripExitS: Double = 0.20,
    val midpointLimitM: Double = 0.18,
    val spacingToleranceM: Double = 0.16,
    val fistCurl: Double = 0.65,
    val extendedCurl: Double = 0.25,
    val throttleDeadRad: Double = 0.035,
    val throttleFullRad: Double = 0.65,
    val brakeContactM: Double = 0.065,
    val brakeDwellS: Double = 0.12,
    val brakeTravelM: Double = 0.055,
    val brakeCurlRange: Double = 0.65,
    val brakeOffsetM: Double = 0.09,
    val steerDeadRad: Double = 0.025,
    val steerFullRad: Double = 0.50,
    val leanDeadRad: Double = 0.035,
    val leanFullRad: Double = 0.60,
    val handleYawLimitRad: Double = 0.65,
    val handleRollLimitRad: Double = 0.85,
    val bodyDeadM: Double = 0.025,
    val bodyFullM: Double = 0.25,
    val filterCutoffHz: Double = 2.0,
    val filterBeta: Double = 0.08,
    val inputTimeoutS: Double = 0.15,
    val lossDecayS: Double = 0.50,
    val bothLostPauseS: Double = 1.0,
    val seatWidthM: Double = 0.30,
    val seatDepthM: Double = 0.42,
    val seatGrabRangeM: Double = 0.24,
    val defaultSeatHeightM: Double = 0.48,
    val hipAboveSeatM: Double = 0.04,
    val headAboveHipM: Double = 0.75,
    val seatBehindHeadM: Double = 0.15,
    val handleHeightM: Double = 0.30,
    val handleForwardM: Double = 0.55,
    val handleHalfWidthM: Double = 0.23,
    val visualLeanRad: Double = 0.20,
    val visualSteerRad: Double = 0.25
) {
    fun validate(): InputConfig {
        val values = inputJson.parseToJsonElement(encode()).toString()
        require(!values.contains("NaN") && !values.contains("Infinity"))
        require(gripEnterM > 0 && gripExitM > gripEnterM)
        require(gripEnterS >= 0 && gripExitS >= 0 && brakeDwellS >= 0)
        require(midpointLimitM > 0 && spacingToleranceM > 0)
        require(fistCurl in 0.0..1.0 && extendedCurl in 0.0..fistCurl)
        validateRanges()
        validateDimensions()
        return this
    }
    private fun validateRanges() {
        require(throttleDeadRad >= 0 && throttleFullRad > throttleDeadRad)
        require(steerDeadRad >= 0 && steerFullRad > steerDeadRad)
        require(leanDeadRad >= 0 && leanFullRad > leanDeadRad)
        require(handleYawLimitRad >= steerFullRad && handleRollLimitRad >= leanFullRad)
        require(bodyDeadM >= 0 && bodyFullM > bodyDeadM)
        require(brakeContactM > 0 && brakeTravelM > 0 && brakeCurlRange > 0 && brakeOffsetM > 0)
        require(filterCutoffHz > 0 && filterBeta >= 0 && inputTimeoutS > 0)
        require(lossDecayS > 0 && bothLostPauseS >= lossDecayS)
    }
    private fun validateDimensions() {
        require(listOf(seatWidthM, seatDepthM, seatGrabRangeM, defaultSeatHeightM,
            hipAboveSeatM, headAboveHipM, seatBehindHeadM, handleHeightM, handleForwardM,
            handleHalfWidthM, visualLeanRad, visualSteerRad).all { it > 0 })
    }
    fun encode() = inputJson.encodeToString(this)
    companion object { fun decode(s: String) = inputJson.decodeFromString<InputConfig>(s).validate() }
}

@Serializable
data class HandPose(
    val palm: Pose,
    val curls: List<Double>,
    val fingerTips: List<Vec3>,
    val joints: List<Pose> = emptyList(),
    val pinchValue: Double? = null,
    val aim: Pose? = null,
    val pinch: Pose? = null,
    val pinchReady: Boolean = false,
    val aimValue: Double? = null,
    val aimReady: Boolean = false
) {
    fun valid() = palm.valid() && curls.size == 4 && curls.all { it.isFinite() && it in 0.0..1.0 } &&
        fingerTips.size == 2 && fingerTips.all { it.valid() } && joints.all { it.valid() }
}

@Serializable
data class HandSnapshot(
    val timestampS: Double,
    val sampleTimestampS: Double,
    val left: HandPose?,
    val right: HandPose?,
    val hmd: Pose,
    val rawLeft: HandPose? = null,
    val rawRight: HandPose? = null,
    val rawHmd: Pose? = null,
    val worldToScene: Pose = Pose(),
    val providerTimestamp: Long? = null,
    val expectedIntent: RiderIntent? = null,
    val coordinateSpace: String = "scene"
)

@Serializable
data class CalibrationFrame(val snapshot: HandSnapshot, val rig: RigFrame)

@Serializable
enum class InputStatus { CALIBRATING, DRIVING, RELEASED, TRACKING_LOST, PAUSED }

@Serializable
data class RiderIntent(
    val throttle: Double = 0.0,
    val brake: Double = 0.0,
    val steer: Double = 0.0,
    val lean: Double = 0.0,
    val bodyLean: Double = 0.0,
    val leftGrip: Boolean = false,
    val rightGrip: Boolean = false,
    val leftProximity: Double = 0.0,
    val rightProximity: Double = 0.0,
    val leftTracked: Boolean = false,
    val rightTracked: Boolean = false,
    val brakeReady: Boolean = false,
    val throttleArmed: Boolean = false,
    val status: InputStatus = InputStatus.CALIBRATING
)
