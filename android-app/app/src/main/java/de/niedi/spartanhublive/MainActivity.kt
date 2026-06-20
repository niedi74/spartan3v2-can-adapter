package de.niedi.spartanhublive

import android.app.Activity
import android.graphics.Color
import android.os.Bundle
import android.os.SystemClock
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.TextView
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

class MainActivity : Activity() {
    private lateinit var connectionStatus: TextView
    private lateinit var rpmValue: TextView
    private lateinit var lambdaValue: TextView
    private lateinit var bm6Value: TextView
    private lateinit var engineMeta: TextView
    private lateinit var bm6Meta: TextView
    private lateinit var lambdaMeta: TextView
    private lateinit var logMeta: TextView
    private lateinit var hostInput: EditText
    private lateinit var endpointText: TextView
    private lateinit var liveValues: LinearLayout

    @Volatile private var endpoint = "http://192.168.4.1/api/status?client=android-live"
    @Volatile private var lastSuccessMs = 0L
    private var poller: ScheduledExecutorService? = null

    private val okColor = Color.rgb(158, 216, 91)
    private val warningColor = Color.rgb(255, 209, 102)
    private val dangerColor = Color.rgb(255, 107, 107)
    private val textColor = Color.rgb(230, 237, 232)
    private val mutedColor = Color.rgb(156, 169, 159)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)

        connectionStatus = findViewById(R.id.connectionStatus)
        rpmValue = findViewById(R.id.rpmValue)
        lambdaValue = findViewById(R.id.lambdaValue)
        bm6Value = findViewById(R.id.bm6Value)
        engineMeta = findViewById(R.id.engineMeta)
        bm6Meta = findViewById(R.id.bm6Meta)
        lambdaMeta = findViewById(R.id.lambdaMeta)
        logMeta = findViewById(R.id.logMeta)
        hostInput = findViewById(R.id.hostInput)
        endpointText = findViewById(R.id.endpointText)
        liveValues = findViewById(R.id.liveValues)

        val prefs = getSharedPreferences("hub", MODE_PRIVATE)
        hostInput.setText(prefs.getString("host", "192.168.4.1"))
        applyHost(hostInput.text.toString(), persist = false)

        findViewById<Button>(R.id.apPresetButton).setOnClickListener {
            hostInput.setText("192.168.4.1")
            applyHost("192.168.4.1", persist = true)
        }
        findViewById<Button>(R.id.saveHostButton).setOnClickListener {
            applyHost(hostInput.text.toString(), persist = true)
        }
    }

    override fun onStart() {
        super.onStart()
        startPolling()
    }

    override fun onStop() {
        poller?.shutdownNow()
        poller = null
        super.onStop()
    }

    private fun applyHost(raw: String, persist: Boolean) {
        var host = raw.trim()
        host = host.removePrefix("http://").removePrefix("https://").trimEnd('/')
        if (host.isBlank()) host = "192.168.4.1"
        hostInput.setText(host)
        endpoint = "http://$host/api/status?client=android-live"
        endpointText.text = endpoint.substringBefore('?')
        if (persist) {
            getSharedPreferences("hub", MODE_PRIVATE).edit().putString("host", host).apply()
            lastSuccessMs = 0L
            showConnecting()
        }
    }

    private fun startPolling() {
        if (poller != null) return
        poller = Executors.newSingleThreadScheduledExecutor().also { executor ->
            executor.scheduleWithFixedDelay({ pollOnce() }, 0, 500, TimeUnit.MILLISECONDS)
        }
    }

    private fun pollOnce() {
        var connection: HttpURLConnection? = null
        try {
            connection = URL(endpoint).openConnection() as HttpURLConnection
            connection.connectTimeout = 1800
            connection.readTimeout = 1800
            connection.useCaches = false
            connection.setRequestProperty("X-Device", "android-live")
            if (connection.responseCode != HttpURLConnection.HTTP_OK) {
                throw IllegalStateException("HTTP ${connection.responseCode}")
            }
            val body = connection.inputStream.bufferedReader().use { it.readText() }
            val data = JSONObject(body)
            lastSuccessMs = SystemClock.elapsedRealtime()
            runOnUiThread { showLive(data) }
        } catch (_: Exception) {
            val staleFor = SystemClock.elapsedRealtime() - lastSuccessMs
            if (lastSuccessMs == 0L || staleFor > 2000L) {
                runOnUiThread { showOffline() }
            }
        } finally {
            connection?.disconnect()
        }
    }

    private fun showLive(data: JSONObject) {
        connectionStatus.text = "LIVE"
        connectionStatus.setTextColor(okColor)
        liveValues.alpha = 1f

        rpmValue.text = data.optInt("rpm", 0).toString()
        lambdaValue.text = if (data.optBoolean("valid", false)) {
            String.format(Locale.US, "%.3f", data.optDouble("lambda", 0.0))
        } else "-.---"
        bm6Value.text = if (data.optBoolean("bm6_connected", false)) {
            String.format(Locale.US, "%.2f V", data.optDouble("bm6_voltage", 0.0))
        } else "--.-- V"

        engineMeta.text = String.format(
            Locale.US,
            "ADV %.1f deg  |  MAP %d",
            data.optDouble("advance", 0.0),
            data.optInt("map", 0)
        )
        val bm6Connected = data.optBoolean("bm6_connected", false)
        bm6Meta.text = "BM6 ${data.optInt("bm6_temperature", 0)} C  |  " +
            if (bm6Connected) "verbunden" else "scan/retry"
        bm6Meta.setTextColor(if (bm6Connected) okColor else warningColor)

        val source = data.optString("source", "NONE")
        val testMode = data.optString("lambda_test_mode", "off")
        lambdaMeta.text = "Lambda: $source" + if (testMode != "off") " ($testMode)" else ""
        lambdaMeta.setTextColor(if (data.optBoolean("valid", false)) okColor else warningColor)

        val loggingEnabled = data.optBoolean("hub_feat_log", false)
        val logReady = data.optBoolean("log_ready", false)
        val logError = data.optString("log_error", "")
        logMeta.text = when {
            !loggingEnabled -> "Log: AUS"
            logReady -> "Log: AN  |  ${formatBytes(data.optLong("log_current_bytes", 0))}"
            logError.isNotBlank() -> "Log: FEHLER ($logError)"
            else -> "Log: nicht bereit"
        }
        logMeta.setTextColor(if (loggingEnabled && logReady) okColor else dangerColor)

        val tuneLive = data.optBoolean("tune_connected", false)
        rpmValue.setTextColor(if (tuneLive) textColor else warningColor)
    }

    private fun showConnecting() {
        connectionStatus.text = "VERBINDE..."
        connectionStatus.setTextColor(warningColor)
    }

    private fun showOffline() {
        connectionStatus.text = "OFFLINE"
        connectionStatus.setTextColor(dangerColor)
        liveValues.alpha = 0.42f
        bm6Meta.setTextColor(mutedColor)
        lambdaMeta.setTextColor(mutedColor)
        logMeta.setTextColor(mutedColor)
    }

    private fun formatBytes(bytes: Long): String = when {
        bytes < 1024 -> "$bytes B"
        bytes < 1024 * 1024 -> String.format(Locale.US, "%.1f KB", bytes / 1024.0)
        else -> String.format(Locale.US, "%.2f MB", bytes / (1024.0 * 1024.0))
    }
}
