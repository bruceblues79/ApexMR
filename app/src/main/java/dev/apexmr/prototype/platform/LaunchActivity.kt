package dev.apexmr.prototype.platform

import com.pico.spatial.ui.platform.stub.SpatialLaunchActivity
import android.os.Bundle
import android.content.pm.PackageManager
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

class LaunchActivity : SpatialLaunchActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        if (ContextCompat.checkSelfPermission(this, "com.picovr.permission.HAND_TRACKING") !=
            PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf("com.picovr.permission.HAND_TRACKING"), 10)
        }
    }
}
