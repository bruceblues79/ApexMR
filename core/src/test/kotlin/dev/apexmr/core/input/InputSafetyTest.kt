package dev.apexmr.core.input

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Path

class InputSafetyTest {
    @TempDir lateinit var temporary: Path
    private fun hand(x: Double, curls: List<Double> = List(4) { 0.9 }) =
        HandPose(Pose(Vec3(x, 0.3, -0.55)), curls, List(2) { Vec3(x, 0.3, -0.64) })
    private fun frame(t: Double) = HandSnapshot(t, t, hand(-0.23), hand(0.23), Pose(Vec3(0.0, 0.8, 0.0)))
    private fun engine() = InputEngine(InputConfig()).apply {
        calibrate(frame(0.0), RigFrame())
        repeat(40) { process(frame((it + 1) * 0.02)) }
    }
    @Test fun `both hands lost pause after elapsed deadline even with long sample gap`() {
        val engine = engine()
        val lost = frame(1.0).copy(left = null, right = null)
        assertEquals(InputStatus.TRACKING_LOST, engine.process(lost).status)
        assertEquals(InputStatus.PAUSED, engine.process(lost.copy(timestampS = 2.1, sampleTimestampS = 2.1)).status)
    }
    @Test fun `single extended finger cannot prepare or activate brake`() {
        val engine = engine()
        repeat(50) {
            val out = engine.process(frame(1.0 + it * 0.02).copy(right = hand(0.23, listOf(0.05, 0.9, 0.9, 0.9))))
            assertEquals(0.0, out.brake)
            assertEquals(0.0, out.throttle)
            assertFalse(out.brakeReady)
        }
    }
    @Test fun `handle rotation beyond permitted range exits grip`() {
        val engine = engine()
        val q = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -1.8)
        val middle = Vec3(0.0, 0.3, -0.55)
        var out = RiderIntent()
        repeat(40) {
            val f = frame(1.0 + it * 0.02)
            out = engine.process(f.copy(
                left = f.left!!.copy(palm = Pose(middle + q.rotate(Vec3(-0.23)), q)),
                right = f.right!!.copy(palm = Pose(middle + q.rotate(Vec3(0.23)), q))
            ))
        }
        assertFalse(out.leftGrip)
        assertFalse(out.rightGrip)
        assertEquals(InputStatus.RELEASED, out.status)
        assertEquals(0.0, out.throttle)
    }
    @Test fun `replay checks actual recorded outputs and rejects altered output`() {
        val engine = engine()
        val f = frame(1.0)
        val expected = engine.process(f)
        val base = engine.calibration!!
        val warmup = (1..40).map { frame(it * 0.02) }
        val recording = Recording(InputConfig(), base, warmup + f.copy(expectedIntent = expected), "synthetic")
        assertTrue(ReplayRunner.verify(recording))
        assertFalse(ReplayRunner.verify(recording.copy(frames = warmup + f.copy(expectedIntent = expected.copy(throttle = 1.0)))))
    }
    @Test fun `recorded proximity tolerates cross platform rounding but detects meaningful changes`() {
        val engine = engine()
        val frame = frame(1.0)
        val expected = engine.process(frame)
        val warmup = (1..40).map { frame(it * 0.02) }
        val recording = Recording(InputConfig(), engine.calibration!!,
            warmup + frame.copy(expectedIntent = expected.copy(leftProximity = expected.leftProximity + 1e-14)), "synthetic")
        assertTrue(ReplayRunner.verify(recording))
        assertFalse(ReplayRunner.verify(recording.copy(frames = warmup +
            frame.copy(expectedIntent = expected.copy(leftProximity = expected.leftProximity + 0.1)))))
    }

    @Test fun `comparison exports new outputs while strict replay rejects older algorithm outputs`() {
        val engine = engine()
        val recording = Recording(InputConfig(), engine.calibration!!,
            listOf(frame(1.0).copy(expectedIntent = RiderIntent(throttle = 1.0))), "swan-real")
        val input = temporary.resolve("recording.jsonl").toFile()
        val output = temporary.resolve("intents.jsonl").toFile()
        input.writeText(recording.encode())
        assertThrows(IllegalArgumentException::class.java) {
            ReplayRunner.main(arrayOf(input.path, output.path))
        }
        assertFalse(output.exists())
        ReplayRunner.main(arrayOf(input.path, output.path, "--compare"))
        val actual = inputJson.decodeFromString<RiderIntent>(output.readText().trim())
        assertEquals(ReplayRunner.run(recording).single(), actual)
        assertFalse(ReplayRunner.verify(recording))
        assertThrows(IllegalArgumentException::class.java) {
            ReplayRunner.main(arrayOf(input.path, output.path, "--skip"))
        }
    }
}
