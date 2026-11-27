package dev.apexmr.core.input

import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import java.io.File
import kotlin.math.abs

@Serializable
data class Recording(
    val config: InputConfig,
    val calibration: CalibrationFrame,
    val frames: List<HandSnapshot>,
    val source: String,
    val schema: Int = 1
) {
    fun encode(): String {
        val header = copy(frames = emptyList())
        return (listOf(inputJson.encodeToString(header)) + frames.map { inputJson.encodeToString(it) })
            .joinToString("\n", postfix = "\n")
    }
    companion object {
        fun decode(text: String): Recording {
            val lines = text.lineSequence().filter { it.isNotBlank() }.toList()
            require(lines.isNotEmpty())
            val header = inputJson.decodeFromString<Recording>(lines.first())
            require(header.schema == 1)
            header.config.validate()
            return header.copy(frames = lines.drop(1).map { inputJson.decodeFromString<HandSnapshot>(it) })
        }
    }
}

object ReplayRunner {
    fun run(recording: Recording): List<RiderIntent> {
        val engine = InputEngine(recording.config)
        engine.calibrate(recording.calibration.snapshot, recording.calibration.rig)
        return recording.frames.map { engine.process(it) }
    }
    fun verify(recording: Recording): Boolean {
        val outputs = run(recording)
        return recording.frames.zip(outputs).all { (frame, actual) ->
            val expected = frame.expectedIntent
            expected == null || matches(actual, expected)
        }
    }
    private fun matches(actual: RiderIntent, expected: RiderIntent): Boolean {
        val a = listOf(actual.throttle, actual.brake, actual.steer, actual.lean, actual.bodyLean,
            actual.leftProximity, actual.rightProximity)
        val e = listOf(expected.throttle, expected.brake, expected.steer, expected.lean, expected.bodyLean,
            expected.leftProximity, expected.rightProximity)
        val flags = actual.copy(throttle = expected.throttle, brake = expected.brake, steer = expected.steer,
            lean = expected.lean, bodyLean = expected.bodyLean,
            leftProximity = expected.leftProximity, rightProximity = expected.rightProximity)
        return flags == expected && a.zip(e).all { (x, y) -> abs(x - y) <= 1e-6 }
    }
    @JvmStatic
    fun main(args: Array<String>) {
        require(args.size == 2) { "ReplayRunner <input.jsonl> <intents.jsonl>" }
        val recording = Recording.decode(File(args[0]).readText())
        require(verify(recording)) { "Recorded intents do not match replay" }
        File(args[1]).writeText(run(recording).joinToString("\n", postfix = "\n") { inputJson.encodeToString(it) })
        println("Replayed ${recording.frames.size} frames; source=${recording.source}")
    }
}
