package dev.apexmr.core.input

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.PI

class InputEngineTest {
    private val params = InputConfig()
    private val neutralLeft = Vec3(-0.23, 0.30, -0.55)
    private val neutralRight = Vec3(0.23, 0.30, -0.55)
    private val head = Pose(Vec3(0.0, 0.80, 0.0))
    private fun hand(position: Vec3, rotation: Rotation = Rotation(), curls: List<Double> = List(4) { 0.9 },
                     tips: List<Vec3> = listOf(Vec3(0.23, 0.30, -0.64), Vec3(0.23, 0.30, -0.64))) =
        HandPose(Pose(position, rotation), curls, tips)
    private fun frame(t: Double, left: HandPose? = hand(neutralLeft), right: HandPose? = hand(neutralRight),
                      hmd: Pose = head) = HandSnapshot(t, t, left, right, hmd)
    private fun engine() = InputEngine(params).apply {
        calibrate(frame(0.0), RigFrame())
        repeat(30) { process(frame((it + 1) * 0.02)) }
    }
    private fun settle(engine: InputEngine, start: Double = 1.0, left: HandPose? = hand(neutralLeft),
                       right: HandPose? = hand(neutralRight), hmd: Pose = head): RiderIntent {
        var result = RiderIntent()
        repeat(50) { result = engine.process(frame(start + it * 0.02, left, right, hmd)) }
        return result
    }

    @Test fun `neutral fist stays zero and grip enters after dwell`() {
        val e = engine()
        val result = settle(e)
        assertTrue(result.leftGrip && result.rightGrip)
        assertEquals(0.0, result.throttle, 1e-8)
        assertEquals(0.0, result.steer, 1e-8)
        assertEquals(0.0, result.lean, 1e-8)
    }

    @Test fun `pair acquisition uses one dwell and does not require both palms moving`() {
        val e = InputEngine(params).apply { calibrate(frame(0.0), RigFrame()) }
        repeat(7) {
            assertEquals(InputStatus.RELEASED, e.process(frame((it + 1) * 0.02)).status)
        }
        assertEquals(InputStatus.DRIVING, e.process(frame(0.16)).status)
        val r = settle(e, left = hand(neutralLeft),
            right = hand(neutralRight + Vec3(0.0, -0.04, 0.0)))
        assertEquals(InputStatus.DRIVING, r.status)
        assertTrue(r.lean > 0.0)
    }

    @Test fun `placement translation preserves all relative parts and rig transforms`() {
        val rig = RigFrame(Vec3(1.0, 0.5, 2.0), PI / 2)
        val local = Vec3(0.2, 0.3, -0.5)
        assertEquals(local.x, rig.toLocal(rig.toWorld(local)).x, 1e-8)
        assertEquals(local.z, rig.toLocal(rig.toWorld(local)).z, 1e-8)
        val moved = rig.copy(origin = rig.origin + Vec3(2.0, 3.0, 4.0))
        assertEquals(Vec3(2.0, 3.0, 4.0), moved.toWorld(local) - rig.toWorld(local))
    }

    @Test fun `horizontal hand line turns right positive with no lean`() {
        val e = engine()
        val q = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -0.25)
        val mid = (neutralLeft + neutralRight) * 0.5
        val r = settle(e, left = hand(mid + q.rotate(neutralLeft - mid), q),
            right = hand(mid + q.rotate(neutralRight - mid), q))
        assertTrue(r.steer > 0.3)
        assertEquals(0.0, r.lean, 1e-6)
        assertEquals(0.0, r.throttle, 1e-6)
        assertTrue(r.leftGrip && r.rightGrip)
    }

    @Test fun `right lean positive and isolated from steering and throttle`() {
        val e = engine()
        val q = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), 0.25)
        val mid = (neutralLeft + neutralRight) * 0.5
        val r = settle(e, left = hand(mid + q.rotate(neutralLeft - mid), q),
            right = hand(mid + q.rotate(neutralRight - mid), q))
        assertTrue(r.lean > 0.3)
        assertEquals(0.0, r.steer, 1e-6)
        assertEquals(0.0, r.throttle, 1e-6)
    }

    @Test fun `head motion bounded compensation without active steering`() {
        val r = settle(engine(), hmd = Pose(Vec3(0.4, 0.80, 0.0)))
        assertTrue(r.bodyLean > 0.0 && r.bodyLean <= 1.0)
        assertEquals(0.0, r.steer, 1e-8)
        assertEquals(0.0, r.lean, 1e-8)
    }

    @Test fun `translation alone never turns or opens throttle`() {
        val delta = Vec3(0.03, 0.02, 0.02)
        val r = settle(engine(), left = hand(neutralLeft + delta), right = hand(neutralRight + delta))
        assertEquals(0.0, r.steer, 1e-8)
        assertEquals(0.0, r.lean, 1e-8)
        assertEquals(0.0, r.throttle, 1e-8)
    }

    @Test fun `twist gives continuous throttle open palm closes and requires zero`() {
        val e = engine()
        val twist = Rotation.axisAngle(Vec3(1.0, 0.0, 0.0), 0.35)
        assertTrue(settle(e, right = hand(neutralRight, twist)).throttle > 0.3)
        assertEquals(0.0, settle(e, 2.1, right = hand(neutralRight, twist, List(4) { 0.0 })).throttle)
        assertEquals(0.0, settle(e, 3.2, right = hand(neutralRight, twist)).throttle)
        settle(e, 4.3)
        assertTrue(settle(e, 5.4, right = hand(neutralRight, twist)).throttle > 0.3)
    }

    @Test fun `extended fingers cut throttle but brake requires contact then pull`() {
        val e = engine()
        val extended = listOf(0.05, 0.05, 0.9, 0.9)
        val outside = hand(neutralRight, curls = extended, tips = List(2) { Vec3(0.23, 0.5, -0.8) })
        val r = settle(e, right = outside)
        assertEquals(0.0, r.brake)
        assertEquals(0.0, r.throttle)
        val contact = hand(neutralRight, curls = extended)
        assertEquals(0.0, settle(e, 2.1, right = contact).brake)
        val pull = hand(neutralRight, curls = listOf(0.55, 0.55, 0.9, 0.9),
            tips = List(2) { Vec3(0.23, 0.30, -0.59) })
        val braking = settle(e, 3.2, right = pull)
        assertTrue(braking.brake > 0.4)
        assertEquals(0.0, braking.throttle)
        assertTrue(braking.rightGrip)
    }

    @Test fun `ordinary fist does not brake and grip exits with hysteresis`() {
        val e = engine()
        assertEquals(0.0, settle(e).brake)
        assertTrue(e.process(frame(2.1, right = hand(neutralRight + Vec3(0.3, 0.0, 0.0)))).rightGrip)
        assertFalse(settle(e, 2.2, right = hand(neutralRight + Vec3(0.3, 0.0, 0.0))).rightGrip)
    }

    @Test fun `missing stale invalid and reordered data are safe`() {
        val e = engine()
        val r = e.process(frame(1.0, right = null))
        assertEquals(0.0, r.throttle)
        assertEquals(InputStatus.TRACKING_LOST, r.status)
        assertEquals(r, e.process(frame(0.9)))
        val stale = e.process(frame(2.0).copy(sampleTimestampS = 1.0))
        assertEquals(InputStatus.TRACKING_LOST, stale.status)
        assertEquals(0.0, stale.throttle)
        val invalid = settle(e, 2.1, right = hand(Vec3(Double.NaN, 0.0, 0.0)))
        assertEquals(InputStatus.TRACKING_LOST, invalid.status)
        assertEquals(0.0, invalid.throttle)
    }

    @Test fun `pause and resume never restore throttle until zero`() {
        val e = engine()
        e.pause()
        assertEquals(InputStatus.PAUSED, settle(e).status)
        e.resume()
        val twist = Rotation.axisAngle(Vec3(1.0, 0.0, 0.0), 0.35)
        assertEquals(0.0, settle(e, 2.1, right = hand(neutralRight, twist)).throttle)
        settle(e, 3.2)
        assertTrue(settle(e, 4.3, right = hand(neutralRight, twist)).throttle > 0.3)
    }

    @Test fun `configuration round trip validates and replay is deterministic`() {
        assertEquals(params, InputConfig.decode(params.encode()))
        assertThrows(IllegalArgumentException::class.java) { params.copy(gripEnterM = -1.0).validate() }
        assertThrows(IllegalArgumentException::class.java) { params.copy(filterCutoffHz = 0.0).validate() }
        val frames = (0..60).map { frame(it * 0.02) }
        val recording = Recording(params, CalibrationFrame(frame(0.0), RigFrame()), frames, "synthetic")
        val restored = Recording.decode(recording.encode())
        assertEquals(ReplayRunner.run(recording), ReplayRunner.run(restored))
    }

    @Test fun `uncalibrated engine is inert and invalid calibration rejected`() {
        val e = InputEngine(params)
        assertEquals(InputStatus.CALIBRATING, e.process(frame(0.0)).status)
        assertThrows(IllegalArgumentException::class.java) { e.calibrate(frame(1.0, right = null), RigFrame()) }
    }

    @Test fun `rigid bilateral lean stays green with no throttle or steering crosstalk`() {
        for (angle in listOf(-0.75, -0.45, 0.45, 0.75)) {
            val e = engine()
            val q = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), angle)
            val r = settle(e, left = hand(q.rotate(neutralLeft), q),
                right = hand(q.rotate(neutralRight), q))
            assertEquals(InputStatus.DRIVING, r.status)
            assertEquals(0.0, r.leftProximity, 1e-8)
            assertEquals(0.0, r.rightProximity, 1e-8)
            assertEquals(0.0, r.throttle, 1e-8)
            assertEquals(0.0, r.steer, 1e-8)
            assertEquals(kotlin.math.sign(angle), kotlin.math.sign(r.lean))
        }
    }

    @Test fun `either palm leaving blocks control before grip exit dwell and requires pair reacquisition`() {
        val twist = Rotation.axisAngle(Vec3(1.0, 0.0, 0.0), 0.35)
        for (side in listOf("left", "right")) {
            val e = engine()
            assertTrue(settle(e, right = hand(neutralRight, twist)).throttle > 0.3)
            val previous = e.process(frame(2.05, right = hand(neutralRight, twist)))
            val delta = Vec3(0.0, 0.5, 0.0)
            val released = e.process(frame(2.07,
                left = hand(neutralLeft + if (side == "left") delta else Vec3()),
                right = hand(neutralRight + if (side == "right") delta else Vec3(), twist)))
            assertTrue(released.leftGrip && released.rightGrip)
            assertEquals(InputStatus.RELEASED, released.status)
            assertEquals(0.0, released.throttle)
            assertEquals(0.0, released.brake)
            assertFalse(released.throttleArmed)
            assertTrue(kotlin.math.abs(released.lean) <= kotlin.math.abs(previous.lean))
            assertTrue(kotlin.math.abs(released.steer) <= kotlin.math.abs(previous.steer))
            val reentered = e.process(frame(2.09))
            assertEquals(InputStatus.RELEASED, reentered.status)
            assertEquals(InputStatus.DRIVING, settle(e, 2.11).status)
            assertTrue(settle(e, 3.2, right = hand(neutralRight, twist)).throttle > 0.3)
        }
    }

    @Test fun `regripping at open throttle does not arm until neutral wrist`() {
        val e = engine()
        val twist = Rotation.axisAngle(Vec3(1.0, 0.0, 0.0), 0.35)
        assertTrue(settle(e, right = hand(neutralRight, twist)).throttle > 0.3)
        e.process(frame(2.1, left = null, right = hand(neutralRight, twist)))
        assertEquals(0.0, settle(e, 2.2, right = hand(neutralRight, twist)).throttle)
        settle(e, 3.3)
        assertTrue(settle(e, 4.4, right = hand(neutralRight, twist)).throttle > 0.3)
    }

    @Test fun `rigid combined turn and lean shares rendered grip targets and brake coordinates`() {
        val e = engine()
        val roll = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), 0.55)
        val yaw = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -0.3)
        val q = roll * yaw
        val nominal = Vec3(0.0, params.handleHeightM, -params.handleForwardM)
        val center = roll.rotate(nominal)
        fun moved(p: Vec3) = center + q.rotate(p - nominal)
        val l = hand(moved(neutralLeft), q)
        val contact = hand(moved(neutralRight), q, listOf(0.05, 0.05, 0.9, 0.9),
            List(2) { moved(Vec3(0.23, 0.30, -0.64)) })
        val touching = settle(e, left = l, right = contact)
        assertEquals(InputStatus.DRIVING, touching.status)
        assertEquals(0.0, touching.leftProximity, 1e-8)
        assertEquals(0.0, touching.rightProximity, 1e-8)
        assertTrue(touching.brakeReady)
        assertEquals(0.0, touching.throttle)
        val feedback = BikeFeedback.solve(e.calibration, frame(2.0, l, contact), touching, params)
        val rendered = feedback.body.rotation.rotate(feedback.handles.position +
            feedback.handles.rotation.rotate(neutralRight - nominal))
        assertEquals(0.0, (rendered - contact.palm.position).length(), 1e-8)
        val pull = hand(moved(neutralRight), q, listOf(0.55, 0.55, 0.9, 0.9),
            List(2) { moved(Vec3(0.23, 0.30, -0.59)) })
        assertTrue(settle(e, 2.1, left = l, right = pull).brake > 0.4)
    }
}
