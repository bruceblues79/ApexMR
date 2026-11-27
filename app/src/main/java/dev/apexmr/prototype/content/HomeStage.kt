package dev.apexmr.prototype.content

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.AccessibilityComponent
import com.pico.spatial.ui.design.Button
import com.pico.spatial.ui.design.Text
import com.pico.spatial.ui.foundation.content.SpatialView
import com.pico.spatial.ui.foundation.content.setAccessibilityActions
import dev.apexmr.prototype.input.TrackingAdapter
import dev.apexmr.prototype.input.ValidationSession
import kotlinx.coroutines.isActive

@Composable
fun HomeStage() {
    val context = LocalContext.current
    val session = remember { ValidationSession(context) }
    val root = remember { Entity() }
    val scene = remember { BikeScene(root, session.config) }
    val adapter = remember { TrackingAdapter(root, context) }
    val lifecycle = LocalLifecycleOwner.current.lifecycle
    var active by remember { mutableStateOf(true) }
    var ready by remember { mutableStateOf(false) }

    DisposableEffect(lifecycle) {
        adapter.start()
        val observer = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_PAUSE -> { active = false; session.lifecyclePause(); adapter.stop() }
                Lifecycle.Event.ON_RESUME -> { active = true; adapter.start() }
                else -> Unit
            }
        }
        lifecycle.addObserver(observer)
        onDispose {
            lifecycle.removeObserver(observer)
            adapter.stop()
            session.close()
            scene.close()
            root.destroy(true)
        }
    }
    LaunchedEffect(ready, active) {
        if (ready && active) {
            while (isActive) {
                withFrameNanos {
                    session.accept(adapter.snapshot(), adapter.hands.supportState.toString())
                    scene.update(session)
                }
            }
        }
    }
    SpatialView(
        attachments = {
            AttachmentPanel("input-dashboard") {
                ValidationPanel(session, scene)
            }
        }
    ) { content, attachments ->
        content.addEntity(root)
        scene.seat.components.set(AccessibilityComponent("鞍座", "捏住并拖动整车").apply {
            isAccessibilityElement = true
        })
        content.setAccessibilityActions(scene.seat, "input.seat") { }
        scene.instrument.components.set(AccessibilityComponent("输入验证仪表", "摆放、标定及输入状态").apply {
            isAccessibilityElement = true
        })
        content.setAccessibilityActions(scene.instrument, "input.dashboard") { }
        attachments.entity("input-dashboard")?.let { panel ->
            panel.components.set(AccessibilityComponent("输入控制面板", "摆放与标定控制").apply {
                isAccessibilityElement = true
            })
            content.setAccessibilityActions(panel, "input.panel") { }
            scene.instrument.addChild(panel)
        }
        ready = true
    }
}

@Composable
private fun ValidationPanel(session: ValidationSession, scene: BikeScene) {
    var settings by remember { mutableStateOf(false) }
    var additive by remember { mutableStateOf(false) }
    Column(Modifier.width(640.dp).heightIn(max = 700.dp).verticalScroll(rememberScrollState()).padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)) {
        Text("ApexMR · 坐姿输入验证", fontSize = 24.sp)
        Text(session.message, fontSize = 18.sp)
        Text(session.tracking, fontSize = 16.sp)
        Text(session.measurements, fontSize = 14.sp)
        val intent = session.intent
        Text("状态 ${intent.status} · 左握 ${intent.leftGrip} / 右握 ${intent.rightGrip}", fontSize = 18.sp)
        Text("油门 ${fmt(intent.throttle)}   刹车 ${fmt(intent.brake)}   转向 ${fmt(intent.steer)}", fontSize = 20.sp)
        Text("压车 ${fmt(intent.lean)}   上身 ${fmt(intent.bodyLean)}", fontSize = 20.sp)
        Text("油门使能 ${intent.throttleArmed} · 搭杆确认 ${intent.brakeReady}", fontSize = 16.sp)
        session.latest?.right?.let { hand ->
            Text("右手卷曲 ${hand.curls.joinToString(" / ") { fmt(it) }}", fontSize = 14.sp)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            Action("确认摆放", session.placing, session::confirmPlacement)
            Action("双手标定", !session.placing, session::calibrate)
            Action("重新摆放", true, session::reposition)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            Action("暂停", true, session::pause)
            Action("继续", !session.placing, session::resume)
            Action(if (session.showJoints) "隐藏关节" else "显示关节") { session.showJoints = !session.showJoints }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            Action(if (session.recording) "停止录制" else "开始录制") {
                if (session.recording) session.stopRecording() else session.startRecording()
            }
            Action(if (settings) "收起参数" else "调试参数") { settings = !settings }
        }
        if (settings) {
            val config = session.config
            Text("调参后需重新标定；距离 m / 角度 rad / 时间 s", fontSize = 16.sp)
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Action("握持更严格") { session.changeConfig(config.copy(gripEnterM = (config.gripEnterM - 0.01).coerceAtLeast(0.03))) }
                Action("握持更宽松") { session.changeConfig(config.copy(gripEnterM = (config.gripEnterM + 0.01).coerceAtMost(config.gripExitM - 0.01))) }
            }
            Text("进入 ${fmt(config.gripEnterM)} m / 退出 ${fmt(config.gripExitM)} m")
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Action("滤波更稳") { session.changeConfig(config.copy(filterCutoffHz = (config.filterCutoffHz - 0.5).coerceAtLeast(0.5))) }
                Action("响应更快") { session.changeConfig(config.copy(filterCutoffHz = (config.filterCutoffHz + 0.5).coerceAtMost(10.0))) }
            }
            Text("滤波 ${fmt(config.filterCutoffHz)} Hz / 油门满量程 ${fmt(config.throttleFullRad)} rad")
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Action("油门更灵敏") { session.changeConfig(config.copy(throttleFullRad = (config.throttleFullRad - 0.05).coerceAtLeast(0.15))) }
                Action("油门更缓") { session.changeConfig(config.copy(throttleFullRad = (config.throttleFullRad + 0.05).coerceAtMost(1.2))) }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Action("保存参数", true, session::saveConfig)
                Action("导入参数", true, session::importConfig)
                Action(if (additive) "实色提示" else "Add 双层测试") {
                    additive = !additive; scene.additive = additive
                }
            }
        }
    }
}

@Composable
private fun Action(label: String, enabled: Boolean = true, onClick: () -> Unit) {
    Button(onClick = onClick, enabled = enabled,
        modifier = Modifier.heightIn(min = 48.dp).semantics { contentDescription = label }) {
        Text(label, fontSize = 16.sp)
    }
}
private fun fmt(value: Double) = "%.2f".format(value)
