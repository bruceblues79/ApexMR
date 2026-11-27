package dev.apexmr.core.input

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.nio.file.Path

class ReplayFixturesTest {
    private fun recording(name: String): Recording {
        val text = requireNotNull(javaClass.getResource("/input/synthetic/$name.jsonl")).readText()
        return Recording.decode(text)
    }
    @Test fun `all synthetic fixtures replay deterministically with expected signs and safety`() {
        val names = listOf("neutral", "steer-left", "steer-right", "lean-left", "lean-right",
            "throttle", "brake", "left-lost", "both-lost")
        val output = names.associateWith { name ->
            val input = recording(name)
            val first = ReplayRunner.run(input)
            assertEquals(first, ReplayRunner.run(Recording.decode(input.encode())))
            first
        }
        assertTrue(output.getValue("neutral").last().leftGrip)
        assertEquals(0.0, output.getValue("neutral").last().throttle)
        assertTrue(output.getValue("steer-left").last().steer < -0.3)
        assertTrue(output.getValue("steer-right").last().steer > 0.3)
        assertTrue(output.getValue("lean-left").last().lean < -0.3)
        assertTrue(output.getValue("lean-right").last().lean > 0.3)
        assertTrue(output.getValue("throttle").maxOf { it.throttle } > 0.5)
        assertEquals(0.0, output.getValue("throttle").last().throttle)
        assertTrue(output.getValue("brake").last().brake > 0.5)
        assertEquals(InputStatus.TRACKING_LOST, output.getValue("left-lost").last().status)
        assertEquals(InputStatus.PAUSED, output.getValue("both-lost").last().status)
    }
    @Test fun `replay CLI creates actual intents file`(@TempDir directory: Path) {
        val source = directory.resolve("source.jsonl").toFile()
        val output = directory.resolve("output.jsonl").toFile()
        source.writeText(recording("brake").encode())
        ReplayRunner.main(arrayOf(source.path, output.path))
        val intents = output.readLines().map { inputJson.decodeFromString<RiderIntent>(it) }
        assertEquals(200, intents.size)
        assertTrue(intents.last().brake > 0.5)
        assertThrows(IllegalArgumentException::class.java) { ReplayRunner.main(emptyArray()) }
        assertThrows(IllegalArgumentException::class.java) { Recording.decode("") }
    }
}
