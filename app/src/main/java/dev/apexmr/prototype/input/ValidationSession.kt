package dev.apexmr.prototype.input

import android.content.Context
import android.os.SystemClock
import android.util.Log
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import dev.apexmr.core.input.*
import kotlinx.serialization.encodeToString
import java.io.File
import java.util.concurrent.Executors
import kotlin.math.atan2

class ValidationSession(private val context: Context) {
    private val directory = File(context.getExternalFilesDir(null), "input-validation").apply { mkdirs() }
    private val configFile = File(directory, "input-config.json")
    private val io = Executors.newSingleThreadExecutor()
    private var configError: String? = null
    var config by mutableStateOf(loadConfig())
        private set
    var engine = InputEngine(config)
        private set
    var placement = SeatPlacement(RigFrame(Vec3(0.0, config.defaultSeatHeightM, 0.0)), config.seatGrabRangeM)
        private set
    var rig by mutableStateOf(placement.rig)
        private set
    var placing by mutableStateOf(true)
        private set
    var intent by mutableStateOf(RiderIntent())
        private set
    var message by mutableStateOf(configError ?: "坐稳后，捏住鞍座调整整车位置")
        private set
    var showJoints by mutableStateOf(false)
    var recording by mutableStateOf(false)
        private set
    var tracking by mutableStateOf("等待手部追踪")
        private set
    var measurements by mutableStateOf("尚未采集")
        private set
    var latest: HandSnapshot? = null
        private set
    private var placementHead: Pose? = null
    private var writer: java.io.BufferedWriter? = null
    private var sampleCount = 0L
    private var validCount = 0L
    private var beginS = 0.0
    private var lastUiS = 0.0
    private var lastLogS = 0.0
    private var ageSum = 0.0
    private var processSumMs = 0.0
    private var recordingFrames = 0L
    private var initialPositioned = false

    fun accept(frame: HandSnapshot, support: String) {
        val startNs = SystemClock.elapsedRealtimeNanos()
        latest = frame
        if (placing && !initialPositioned && frame.hmd.valid()) {
            val forward = frame.hmd.rotation.rotate(Vec3(0.0, 0.0, -1.0))
            val yaw = atan2(-forward.x, -forward.z)
            val position = frame.hmd.position + Vec3(0.0, -config.headAboveHipM, 0.0) -
                Vec3(forward.x, 0.0, forward.z).normalized() * config.seatBehindHeadM
            placement = SeatPlacement(RigFrame(position, yaw), config.seatGrabRangeM)
            rig = placement.rig
            initialPositioned = true
        }
        if (beginS == 0.0) beginS = frame.timestampS
        sampleCount++
        val valid = frame.left?.valid() == true && frame.right?.valid() == true &&
            frame.timestampS - frame.sampleTimestampS in 0.0..config.inputTimeoutS
        if (valid) validCount++
        ageSum += (frame.timestampS - frame.sampleTimestampS).coerceAtLeast(0.0)
        if (placing) {
            val hand = listOfNotNull(frame.left, frame.right).firstOrNull {
                it.pinchReady && (it.pinchValue ?: 0.0) > 0.80
            }
            rig = placement.update(hand?.pinch?.position ?: hand?.palm?.position, hand != null)
        } else if (configError == null) {
            intent = engine.process(frame.inRig(rig))
        }
        if (recording) {
            val line = inputJson.encodeToString(frame.inRig(rig).copy(expectedIntent = intent))
            io.execute { writer?.append(line)?.append('\n') }
            recordingFrames++
        }
        processSumMs += (SystemClock.elapsedRealtimeNanos() - startNs) / 1e6
        if (frame.timestampS - lastUiS >= 0.10) {
            lastUiS = frame.timestampS
            tracking = "$support · 左${if (frame.left != null) "可用" else "缺失"} / 右${if (frame.right != null) "可用" else "缺失"}"
            val duration = (frame.timestampS - beginS).coerceAtLeast(0.01)
            measurements = "渲染 ${"%.1f".format(sampleCount / duration)} Hz · 双手有效 ${"%.1f".format(100.0 * validCount / sampleCount)}% · " +
                "采样龄 ${"%.1f".format(ageSum * 1000 / sampleCount)} ms · 处理 ${"%.2f".format(processSumMs / sampleCount)} ms"
        }
        if (frame.timestampS - lastLogS >= 5.0) {
            lastLogS = frame.timestampS
            Log.i("ApexMRInput", "support=$support state=${intent.status} placing=$placing joints=${frame.left?.joints?.size}/${frame.right?.joints?.size} $measurements")
        }
    }

    fun confirmPlacement() {
        val frame = latest ?: return
        if (!frame.hmd.valid()) { message = "头部数据无效"; return }
        placement.confirm()
        rig = placement.rig
        placementHead = frame.hmd
        placing = false
        message = "双手自然虚握车把，保持零腕位，然后点击标定"
    }

    fun calibrate() {
        if (configError != null) { message = configError!!; return }
        stopRecording()
        val frame = latest
        if (placing || frame?.left?.valid() != true || frame.right?.valid() != true ||
            frame.timestampS - frame.sampleTimestampS !in 0.0..config.inputTimeoutS) {
            message = "先确认摆放，并让左右手进入追踪范围"; return
        }
        val left = rig.toLocal(frame.left!!.palm.position)
        val right = rig.toLocal(frame.right!!.palm.position)
        val leftHandle = Vec3(-config.handleHalfWidthM, config.handleHeightM, -config.handleForwardM)
        val rightHandle = Vec3(config.handleHalfWidthM, config.handleHeightM, -config.handleForwardM)
        if ((left - leftHandle).length() > config.gripExitM || (right - rightHandle).length() > config.gripExitM) {
            message = "双手靠近左右握把后再标定"; return
        }
        engine.calibrate(frame.copy(hmd = placementHead ?: frame.hmd).inRig(rig), rig)
        message = "已标定：转把 / 压车 / 握拳转腕 / 双指搭杆拉刹"
    }

    fun pause() { engine.pause(); stopRecording(); message = "已暂停，继续前先回到零腕位" }
    fun resume() { engine.resume(); message = "已继续，回零腕位后给油" }
    fun reposition() {
        pause()
        placement = SeatPlacement(rig, config.seatGrabRangeM)
        placing = true
        message = "捏住鞍座调整整车；确认后重新标定"
    }
    fun changeConfig(next: InputConfig) {
        stopRecording()
        config = next.validate()
        configError = null
        engine = InputEngine(config)
        message = "参数已调整，需重新标定"
    }
    fun saveConfig() {
        val text = config.encode()
        io.execute { configFile.writeText(text) }
        message = "已保存 input-config.json，重启生效"
    }
    fun importConfig() {
        try { changeConfig(InputConfig.decode(configFile.readText())); message = "配置已导入，请重新标定" }
        catch (e: IllegalArgumentException) { message = "配置无效：${e.message}" }
        catch (e: java.io.IOException) { message = "配置读取失败：${e.message}" }
    }
    fun startRecording() {
        val calibration = engine.calibration
        if (calibration == null || placing || intent.status == InputStatus.PAUSED) {
            message = "先完成标定并继续，再开始录制"; return
        }
        if (recording) return
        engine.calibrate(calibration.snapshot, calibration.rig)
        val file = File(directory, "swan-${System.currentTimeMillis()}.jsonl")
        val header = inputJson.encodeToString(Recording(config, calibration, emptyList(), "swan-real"))
        io.execute {
            writer = file.bufferedWriter()
            writer?.append(header)?.append('\n')
        }
        recordingFrames = 0
        recording = true
        message = "录制中：${file.name}；先回零腕位并重新稳定握持"
    }
    fun stopRecording() {
        if (!recording) return
        recording = false
        io.execute { writer?.close(); writer = null }
        message = "录制已保存，共 $recordingFrames 帧"
    }
    fun lifecyclePause() { pause(); io.execute { File(directory, "runtime-metrics.txt").writeText(measurements) } }
    fun close() { stopRecording(); io.shutdown() }

    private fun loadConfig(): InputConfig {
        return try {
            if (configFile.exists()) InputConfig.decode(configFile.readText())
            else InputConfig.decode(context.assets.open("input-defaults.json").bufferedReader().use { it.readText() })
                .also { configFile.writeText(it.encode()) }
        } catch (e: IllegalArgumentException) {
            configError = "配置无效，输入已禁用；修正配置后导入"
            Log.e("ApexMRInput", "Invalid config", e); InputConfig()
        } catch (e: java.io.IOException) {
            configError = "配置读取失败，输入已禁用；修正配置后导入"
            Log.e("ApexMRInput", "Config IO", e); InputConfig()
        }
    }
}
