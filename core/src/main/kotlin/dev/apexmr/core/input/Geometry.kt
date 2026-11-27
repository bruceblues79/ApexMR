package dev.apexmr.core.input

import kotlinx.serialization.Serializable
import kotlin.math.*

@Serializable
data class Vec3(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0) {
    operator fun plus(v: Vec3) = Vec3(x + v.x, y + v.y, z + v.z)
    operator fun minus(v: Vec3) = Vec3(x - v.x, y - v.y, z - v.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    fun dot(v: Vec3) = x * v.x + y * v.y + z * v.z
    fun cross(v: Vec3) = Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x)
    fun length() = sqrt(dot(this))
    fun normalized() = this * (1.0 / length().coerceAtLeast(1e-12))
    fun valid() = x.isFinite() && y.isFinite() && z.isFinite()
}

@Serializable
data class Rotation(val x: Double = 0.0, val y: Double = 0.0, val z: Double = 0.0, val w: Double = 1.0) {
    operator fun times(q: Rotation) = Rotation(
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w,
        w * q.w - x * q.x - y * q.y - z * q.z
    )
    fun inverse(): Rotation {
        val n = x * x + y * y + z * z + w * w
        return Rotation(-x / n, -y / n, -z / n, w / n)
    }
    fun rotate(v: Vec3): Vec3 {
        val q = this * Rotation(v.x, v.y, v.z, 0.0) * inverse()
        return Vec3(q.x, q.y, q.z)
    }
    fun valid() = listOf(x, y, z, w).all { it.isFinite() } && x * x + y * y + z * z + w * w > 1e-12
    companion object {
        fun axisAngle(axis: Vec3, angle: Double): Rotation {
            val v = axis.normalized() * sin(angle * 0.5)
            return Rotation(v.x, v.y, v.z, cos(angle * 0.5))
        }
    }
}

@Serializable
data class Pose(val position: Vec3 = Vec3(), val rotation: Rotation = Rotation()) {
    fun valid() = position.valid() && rotation.valid()
}

@Serializable
data class RigFrame(val origin: Vec3 = Vec3(), val yawRad: Double = 0.0) {
    fun rotation() = Rotation.axisAngle(Vec3(0.0, 1.0, 0.0), yawRad)
    fun toLocal(p: Vec3) = rotation().inverse().rotate(p - origin)
    fun toWorld(p: Vec3) = origin + rotation().rotate(p)
}
