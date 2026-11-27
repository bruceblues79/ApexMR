package dev.apexmr.core.input

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class HandGeometryTest {
    @Test fun `straight finger zero and folded finger high curl`() {
        assertEquals(0.0, HandGeometry.curl(Vec3(), Vec3(0.0, 0.0, -0.04), Vec3(0.0, 0.0, -0.08))!!, 1e-8)
        assertEquals(1.0, HandGeometry.curl(Vec3(), Vec3(0.0, 0.0, -0.04), Vec3(0.0, 0.0, 0.0))!!, 1e-8)
        assertNull(HandGeometry.curl(Vec3(), Vec3(), Vec3()))
    }

    @Test fun `seat pinch holds initial offset and releases without jumping`() {
        val placement = SeatPlacement(RigFrame(Vec3(0.0, 0.48, 0.0)), 0.24)
        val point = Vec3(0.1, 0.48, 0.0)
        assertEquals(Vec3(0.0, 0.48, 0.0), placement.update(point, true).origin)
        val moved = placement.update(point + Vec3(0.2, 0.1, 0.1), true).origin
        assertEquals(0.2, moved.x, 1e-8)
        assertEquals(0.58, moved.y, 1e-8)
        assertEquals(0.1, moved.z, 1e-8)
        assertEquals(moved, placement.update(Vec3(9.0), false).origin)
        placement.confirm()
        assertEquals(moved, placement.update(Vec3(9.0), true).origin)
    }

    @Test fun `pinch away from seat cannot begin moving`() {
        val placement = SeatPlacement(RigFrame(), 0.24)
        assertEquals(RigFrame(), placement.update(Vec3(1.0, 0.0, 0.0), true))
        assertFalse(placement.dragging)
    }

    private val config = InputConfig()
    private val mid = Vec3(0.0, config.handleHeightM, -config.handleForwardM)
    private fun hands(center: Vec3 = mid, rotation: Rotation = Rotation(), halfWidth: Double = config.handleHalfWidthM): HandSnapshot {
        fun hand(side: Double) = HandPose(
            Pose(center + rotation.rotate(Vec3(side * halfWidth, 0.0, 0.0))),
            List(4) { 0.9 }, List(2) { center })
        return HandSnapshot(1.0, 1.0, hand(-1.0), hand(1.0), Pose(Vec3(0.0, 0.8, 0.0)), coordinateSpace = "rig")
    }
    private val driving = RiderIntent(leftGrip = true, rightGrip = true, status = InputStatus.DRIVING)
    private fun feedback(frame: HandSnapshot, base: HandSnapshot = hands(), intent: RiderIntent = driving) =
        BikeFeedback.solve(CalibrationFrame(base, RigFrame()), frame, intent, config)

    private fun assertPoint(expected: Vec3, actual: Vec3) {
        assertEquals(expected.x, actual.x, 1e-8)
        assertEquals(expected.y, actual.y, 1e-8)
        assertEquals(expected.z, actual.z, 1e-8)
    }

    @Test fun `rigid handlebars turn once and lean with anchored bike`() {
        for (angle in listOf(-0.5, 0.5)) {
            val yaw = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -0.3)
            val body = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), angle)
            val q = body * yaw
            val frame = hands(mid + Vec3(0.06, 0.02, -0.03), q, 0.25)
            val pose = feedback(frame)
            assertPoint(Vec3(), pose.body.position)
            assertPoint(mid, pose.handles.position)
            for (side in listOf(-1.0, 1.0)) {
                val grip = pose.handles.position + pose.handles.rotation.rotate(
                    Vec3(side * config.handleHalfWidthM, 0.0, 0.0))
                val rendered = pose.body.position + pose.body.rotation.rotate(grip)
                assertPoint(body.rotate(mid + yaw.rotate(Vec3(side * config.handleHalfWidthM, 0.0, 0.0))), rendered)
            }
            assertPoint(body.rotate(Vec3(0.0, 1.0, 0.0)),
                pose.body.rotation.rotate(Vec3(0.0, 1.0, 0.0)))
        }
    }

    @Test fun `lifting moving and spreading palms cannot translate bike or resize handlebars`() {
        val neutral = feedback(hands())
        for (center in listOf(mid, mid + Vec3(0.0, 0.14, 0.0), mid + Vec3(0.1, -0.08, 0.08), mid + Vec3(0.0, 0.9, 0.0))) {
            for (halfWidth in listOf(0.16, 0.23, 0.31, 0.45)) {
                val pose = feedback(hands(center, halfWidth = halfWidth))
                assertPoint(neutral.body.position, pose.body.position)
                assertPoint(neutral.handles.position, pose.handles.position)
                assertEquals(1.0, pose.handleScaleX)
                val left = pose.handles.rotation.rotate(Vec3(-config.handleHalfWidthM, 0.0, 0.0))
                val right = pose.handles.rotation.rotate(Vec3(config.handleHalfWidthM, 0.0, 0.0))
                assertEquals(config.handleHalfWidthM * 2, (right - left).length(), 1e-8)
            }
        }
    }

    @Test fun `neutral head motion and rig world placement do not alter feedback`() {
        val frame = hands()
        val pose = feedback(frame.copy(hmd = Pose(Vec3(0.4, 0.8, 0.0))))
        assertPoint(Vec3(), pose.body.position)
        assertPoint(mid, pose.handles.position)
        assertPoint(Vec3(0.0, 1.0, 0.0), pose.body.rotation.rotate(Vec3(0.0, 1.0, 0.0)))
        val rig = RigFrame(Vec3(1.0, 0.5, 2.0), 1.2)
        fun world(f: HandSnapshot) = f.copy(
            left = f.left!!.copy(palm = Pose(rig.toWorld(f.left.palm.position))),
            right = f.right!!.copy(palm = Pose(rig.toWorld(f.right.palm.position))), coordinateSpace = "scene")
        val placed = BikeFeedback.solve(CalibrationFrame(world(frame), rig), world(frame), driving, config)
        assertPoint(pose.body.position, placed.body.position)
        assertPoint(pose.handles.position, placed.handles.position)
        assertPoint(Vec3(1.0, 0.0, 0.0), placed.handles.rotation.rotate(Vec3(1.0, 0.0, 0.0)))
    }

    @Test fun `feedback ignores missing stale invalid released and paused palms`() {
        val neutral = BikeFeedback.solve(null, null, RiderIntent(), config)
        assertPoint(mid, neutral.handles.position)
        val frame = hands(mid + Vec3(0.1, 0.0, 0.0))
        val invalid = frame.copy(left = frame.left!!.copy(palm = Pose(Vec3(Double.NaN))))
        val short = frame.copy(right = frame.left)
        for (f in listOf(frame.copy(left = null), frame.copy(sampleTimestampS = 0.0), invalid, short)) {
            assertEquals(neutral, feedback(f))
        }
        for (status in listOf(InputStatus.RELEASED, InputStatus.PAUSED, InputStatus.CALIBRATING)) {
            assertEquals(neutral, feedback(frame, intent = driving.copy(status = status)))
        }
        assertEquals(neutral, feedback(frame, intent = driving.copy(leftGrip = false)))
    }

    @Test fun `feedback limits angles and ignores calibration midpoint offset`() {
        val base = hands()
        val q = Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), 1.2) *
            Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -1.0)
        val pose = feedback(hands(mid + Vec3(0.9, 0.0, 0.0), q, 0.45), base)
        assertPoint(Vec3(), pose.body.position)
        assertPoint(mid, pose.handles.position)
        assertPoint(Rotation.axisAngle(Vec3(0.0, 0.0, -1.0), config.handleRollLimitRad).rotate(Vec3(0.0, 1.0, 0.0)),
            pose.body.rotation.rotate(Vec3(0.0, 1.0, 0.0)))
        assertPoint(Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), -config.handleYawLimitRad).rotate(Vec3(1.0, 0.0, 0.0)),
            pose.handles.rotation.rotate(Vec3(1.0, 0.0, 0.0)))
        val shifted = hands(mid + Vec3(0.03, 0.02, 0.01), halfWidth = 0.25)
        val zero = feedback(shifted, shifted)
        assertPoint(Vec3(), zero.body.position)
        assertPoint(mid, zero.handles.position)
        assertEquals(hands(), base)
    }
}
