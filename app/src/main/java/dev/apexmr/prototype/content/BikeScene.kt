package dev.apexmr.prototype.content

import com.pico.spatial.core.ecs.*
import com.pico.spatial.core.ecs.resource.*
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.core.math.Quat
import com.pico.spatial.core.math.Color4
import dev.apexmr.core.input.*
import dev.apexmr.prototype.input.ValidationSession
import dev.apexmr.prototype.input.inRig

class BikeScene(val root: Entity, private val config: InputConfig) {
    val bike = Entity()
    val seat = Entity()
    val instrument = Entity()
    private val body = Entity()
    private val handles = Entity()
    private val resources = mutableListOf<Resource>()
    private val indicators = mutableListOf<Pair<Entity, UnlitMaterial>>()
    private val jointEntities = mutableListOf<Entity>()
    var additive = false

    init {
        root.addChild(bike)
        bike.addChild(body)
        bike.addChild(seat)
        bike.addChild(instrument)
        body.addChild(handles)
        seat.addChild(box("Seat", Vec3(config.seatWidthM, 0.07, config.seatDepthM),
            Vec3(0.0, -config.hipAboveSeatM, 0.0), Color4(0.12f, 0.15f, 0.20f, 1f)))
        body.addChild(box("Tank", Vec3(0.32, 0.23, 0.40), Vec3(0.0, 0.14, -0.31), Color4(0.16f, 0.35f, 0.52f, 1f)))
        body.addChild(box("Dashboard", Vec3(0.26, 0.14, 0.08), Vec3(0.0, 0.38, -0.62), Color4(0.08f, 0.1f, 0.14f, 1f)))
        seat.addChild(box("HipMarker", Vec3(0.02, 0.02, 0.02), Vec3(), Color4(1f, 0.65f, 0f, 1f)))
        handles.components[TransformComponent::class.java]!!.setPosition(Vec3(0.0, config.handleHeightM, -config.handleForwardM).sdk())
        handles.addChild(box("Handlebar", Vec3(0.46, 0.018, 0.018), Vec3(), Color4(0.55f, 0.6f, 0.65f, 1f)))
        listOf(-1.0, 1.0).forEach { side ->
            val position = Vec3(side * config.handleHalfWidthM, 0.0, 0.0)
            handles.addChild(box(if (side < 0) "GripLeft" else "GripRight", Vec3(0.12, 0.035, 0.035), position, Color4(0.12f, 0.12f, 0.12f, 1f)))
            val material = UnlitMaterial.create().apply { setBaseColor(Color4.GREEN) }
            resources.add(material)
            val mesh = MeshResource.createBox(Vec3(0.14, 0.05, 0.05).sdk())
            resources.add(mesh)
            val entity = ModelEntity(mesh, material)
            entity.components[TransformComponent::class.java]!!.setPosition(position.sdk())
            handles.addChild(entity)
            indicators.add(entity to material)
        }
        handles.addChild(box("BrakeLever", Vec3(0.14, 0.012, 0.012),
            Vec3(config.handleHalfWidthM, 0.0, -config.brakeOffsetM), Color4(0.85f, 0.85f, 0.85f, 1f)))
        val jointMesh = MeshResource.createBox(Vector3(0.009f))
        val jointMaterial = UnlitMaterial.create().apply { setBaseColor(Color4(0.2f, 0.8f, 1f, 1f)) }
        resources.add(jointMesh); resources.add(jointMaterial)
        repeat(52) { jointEntities.add(ModelEntity(jointMesh, jointMaterial).also { root.addChild(it); it.enabled = false }) }
    }
    fun update(session: ValidationSession) {
        bike.components[TransformComponent::class.java]!!.apply {
            setPosition(session.rig.origin.sdk()); setQuaternion(session.rig.rotation().sdk())
        }
        val intent = session.intent
        seat.components[AccessibilityComponent::class.java]?.value =
            if (session.placing) "摆放中，捏住拖动" else "摆放已冻结"
        instrument.components[AccessibilityComponent::class.java]?.value =
            "${intent.status}，油门 ${intent.throttle}，刹车 ${intent.brake}"
        val feedback = BikeFeedback.solve(
            if (session.placing) null else session.engine.calibration,
            session.latest?.inRig(session.rig),
            if (session.placing) RiderIntent() else intent, session.config)
        listOf(body, seat).forEach { entity ->
            entity.components[TransformComponent::class.java]!!.apply {
                setPosition(feedback.body.position.sdk())
                setQuaternion(feedback.body.rotation.sdk())
            }
        }
        handles.components[TransformComponent::class.java]!!.apply {
            setPosition(feedback.handles.position.sdk())
            setQuaternion(feedback.handles.rotation.sdk())
            setScaleVector(Vector3(feedback.handleScaleX.toFloat(), 1f, 1f))
        }
        indicators.forEachIndexed { index, (entity, material) ->
            val grip = if (index == 0) intent.leftGrip else intent.rightGrip
            val proximity = if (index == 0) intent.leftProximity else intent.rightProximity
            entity.enabled = grip && !session.placing
            material.setBaseColor(Color4(proximity.toFloat(), (1.0 - proximity).toFloat(), 0.08f, 1f))
            material.setBlendingMode(if (additive) BlendingMode.ADD else BlendingMode.OPAQUE)
            material.setCullingMode(if (additive) MaterialCullingMode.FRONT else MaterialCullingMode.BACK)
        }
        val frame = session.latest
        jointEntities.forEachIndexed { index, entity ->
            val hand = if (index < 26) frame?.left else frame?.right
            val joint = hand?.joints?.getOrNull(index % 26)
            entity.enabled = session.showJoints && joint != null
            if (joint != null) entity.components[TransformComponent::class.java]!!.setPosition(joint.position.sdk())
        }
        instrument.components[TransformComponent::class.java]!!.setPosition(Vector3(0f, 0.66f, -0.88f))
    }
    fun close() { bike.destroy(true); jointEntities.forEach { it.destroy(true) }; resources.forEach { it.close() } }
    private fun box(name: String, size: Vec3, position: Vec3, color: Color4): Entity {
        val mesh = MeshResource.createBox(size.sdk())
        val material = UnlitMaterial.create().apply { setBaseColor(color) }
        resources.add(mesh); resources.add(material)
        return ModelEntity(mesh, material).apply {
            setName(name)
            components[TransformComponent::class.java]!!.setPosition(position.sdk())
        }
    }
}

fun Vec3.sdk() = Vector3(x.toFloat(), y.toFloat(), z.toFloat())
fun Rotation.sdk() = Quat(x.toFloat(), y.toFloat(), z.toFloat(), w.toFloat())
