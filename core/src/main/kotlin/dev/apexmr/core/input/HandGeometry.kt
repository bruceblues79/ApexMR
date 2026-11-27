package dev.apexmr.core.input

import kotlin.math.acos
import kotlin.math.PI

object HandGeometry {
    fun curl(base: Vec3, joint: Vec3, tip: Vec3): Double? {
        val proximal = joint - base
        val distal = tip - joint
        if (!listOf(base, joint, tip).all { it.valid() } ||
            proximal.length() < 0.001 || distal.length() < 0.001) return null
        return (acos(proximal.normalized().dot(distal.normalized()).coerceIn(-1.0, 1.0)) / (PI * 0.80))
            .coerceIn(0.0, 1.0)
    }
}

class SeatPlacement(initial: RigFrame, private val grabRangeM: Double) {
    var rig = initial
        private set
    var dragging = false
        private set
    private var frozen = false
    private var offset = Vec3()

    fun confirm() { frozen = true; dragging = false }
    fun update(point: Vec3?, pinch: Boolean): RigFrame {
        if (frozen) return rig
        if (!pinch || point == null || !point.valid()) { dragging = false; return rig }
        if (!dragging && (point - rig.origin).length() <= grabRangeM) {
            dragging = true
            offset = rig.origin - point
        }
        if (dragging) rig = rig.copy(origin = point + offset)
        return rig
    }
}
