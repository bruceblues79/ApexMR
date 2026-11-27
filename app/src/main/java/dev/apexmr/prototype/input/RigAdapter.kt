package dev.apexmr.prototype.input

import dev.apexmr.core.input.*

fun HandSnapshot.inRig(rig: RigFrame): HandSnapshot {
    fun pose(pose: Pose) = Pose(rig.toLocal(pose.position), rig.rotation().inverse() * pose.rotation)
    fun hand(hand: HandPose?) = hand?.copy(
        palm = pose(hand.palm),
        fingerTips = hand.fingerTips.map { rig.toLocal(it) },
        joints = hand.joints.map { pose(it) },
        pinch = hand.pinch?.let { pose(it) },
        aim = hand.aim?.let { pose(it) }
    )
    return copy(left = hand(left), right = hand(right), hmd = pose(hmd), coordinateSpace = "rig")
}
