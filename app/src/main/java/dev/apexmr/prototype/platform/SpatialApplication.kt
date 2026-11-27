package dev.apexmr.prototype.platform

import android.app.Application
import com.pico.spatial.ui.foundation.dsl.launch
import dev.apexmr.prototype.mainApp

class SpatialApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        launch(::mainApp)
    }
}
