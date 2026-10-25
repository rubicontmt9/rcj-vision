package jp.hobby.rcjvision

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.util.Size
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.CameraInfo
import androidx.camera.core.CameraSelector
import androidx.camera.core.ConcurrentCamera
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.UseCaseGroup
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import java.util.Locale
import java.util.concurrent.Executors
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

private const val TAG = "RcjVision"
private const val MAX_RANGE_MM = 6000.0
private const val ALT_BACK_MS = 1000L   // 背面の滞在時間(切替の立ち上がり込み)
private const val ALT_FRONT_MS = 450L   // 前面の滞在時間

/** 幅に対して高さ4:3(縦長)に固定するコンテナ */
class AspectFrame(ctx: Context) : FrameLayout(ctx) {
    override fun onMeasure(wSpec: Int, hSpec: Int) {
        val w = MeasureSpec.getSize(wSpec)
        super.onMeasure(
            MeasureSpec.makeMeasureSpec(w, MeasureSpec.EXACTLY),
            MeasureSpec.makeMeasureSpec(w * 4 / 3, MeasureSpec.EXACTLY),
        )
    }
}

class Overlay(ctx: Context) : View(ctx) {
    @Volatile var det: Detection? = null
    @Volatile var mark: FloatArray? = null   // 校正でタップした正規化座標
    private val box = Paint().apply { style = Paint.Style.STROKE; strokeWidth = 4f }
    private val txt = Paint().apply { textSize = 30f; style = Paint.Style.FILL; setShadowLayer(4f, 0f, 0f, Color.BLACK) }

    private fun color(c: Cls) = when (c) {
        Cls.BALL -> Color.rgb(255, 140, 0)
        Cls.GOAL_Y -> Color.YELLOW
        Cls.GOAL_B -> Color.rgb(80, 140, 255)
        Cls.ALLY -> Color.MAGENTA
        Cls.OPP -> Color.CYAN
    }

    override fun onDraw(canvas: Canvas) {
        mark?.let {
            box.color = Color.WHITE
            canvas.drawCircle(it[0] * width, it[1] * height, 24f, box)
        }
        val d = det ?: return
        val sx = width.toFloat() / d.w
        val sy = height.toFloat() / d.h
        for (b in d.blobs) {
            box.color = color(b.cls)
            txt.color = box.color
            canvas.drawRect(b.x0 * sx, b.y0 * sy, b.x1 * sx, b.y1 * sy, box)
            val label = if (b.xMm.isNaN()) b.cls.code
            else String.format(Locale.US, "%s %d,%d", b.cls.code, b.xMm.toInt(), b.yMm.toInt())
            canvas.drawText(label, b.x0 * sx, max(30f, b.y0 * sy - 6f), txt)
        }
    }
}

/** カメラ1台分の解析パイプライン */
class CamPipe(val front: Boolean, val ctx: Context, val tracker: Tracker, val imu: Imu) {
    val detector = Detector()
    lateinit var overlay: Overlay
    @Volatile var cameraId: String? = null
    @Volatile var sampleReq: FloatArray? = null          // [nx, ny]
    @Volatile var sampleCb: ((FloatArray) -> Unit)? = null
    @Volatile var detCount = 0
    @Volatile var fps = 0.0
    private var focalPx = 0.0
    private var lastNs = 0L

    private fun computeFocal(iw: Int, ih: Int): Double {
        try {
            val id = cameraId ?: return fallbackFocal(iw)
            val ch = (ctx.getSystemService(Context.CAMERA_SERVICE) as CameraManager).getCameraCharacteristics(id)
            val f = ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: return fallbackFocal(iw)
            val size = ch.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE) ?: return fallbackFocal(iw)
            val sensorAspect = size.width / size.height
            val imgAspect = iw.toFloat() / ih
            // 解析画像が4:3等でセンサーより正方形に近ければ幅方向をクロップ
            return if (imgAspect <= sensorAspect) (f * ih / size.height).toDouble() else (f * iw / size.width).toDouble()
        } catch (e: Exception) {
            Log.w(TAG, "focal fallback: ${e.message}")
            return fallbackFocal(iw)
        }
    }

    private fun fallbackFocal(iw: Int) = (iw / 2.0) / tan(Math.toRadians(65.0 / 2))

    fun analyze(img: ImageProxy) {
        try {
            val nowNs = SystemClock.elapsedRealtimeNanos()
            if (focalPx == 0.0) focalPx = computeFocal(img.width, img.height)
            sampleReq?.let { r ->
                sampleReq = null
                sampleCb?.invoke(detector.sample(img, r[0], r[1]))
            }
            val det = detector.detect(img)
            val up = imu.up
            for (b in det.blobs) {
                val (u, v, z) = when (b.cls) {
                    Cls.BALL, Cls.GOAL_Y, Cls.GOAL_B -> {
                        // 接地点(下端中央)を床面へ。画面下端で切れている場合は距離が信用できない
                        if (b.y1 >= det.h - 2) continue
                        Triple(b.cx.toDouble(), b.y1.toDouble(), 0.0)
                    }
                    else -> Triple(b.cx.toDouble(), b.cy.toDouble(), Geometry.markerHeightM)
                }
                val p = Geometry.project(front, u, v, det.w, det.h, focalPx, up, z) ?: continue
                if (hypot(p[0], p[1]) > MAX_RANGE_MM) continue
                b.xMm = p[0].toFloat(); b.yMm = p[1].toFloat()
                tracker.update(b.cls, if (front) 1 else 0, p[0], p[1], nowNs)
            }
            overlay.det = det
            overlay.postInvalidate()
            detCount = det.blobs.size
            if (lastNs != 0L) fps = fps * 0.8 + 1e9 / (nowNs - lastNs) * 0.2
            lastNs = nowNs
        } finally {
            img.close()
        }
    }
}

class MainActivity : AppCompatActivity() {
    private val tracker = Tracker()
    private lateinit var imu: Imu
    private lateinit var usb: UsbOut
    private lateinit var back: CamPipe
    private lateinit var front: CamPipe
    private lateinit var backView: PreviewView
    private lateinit var frontView: PreviewView
    private lateinit var status: TextView
    private lateinit var usbBtn: Button
    private lateinit var heightBtn: Button
    private lateinit var baudBtn: Button
    private val camExec = Executors.newFixedThreadPool(2)
    private val ui = Handler(Looper.getMainLooper())
    private var camNote = ""
    private var competition = false
    private var cameraProvider: ProcessCameraProvider? = null
    private var altActive = false   // 前後交互切替(同時利用非対応端末向け)
    private var activeCam = 0       // 0=背面 1=前面
    private var calibrating = false
    private var calCls = 0
    private var calTaps = 0
    private var calNote = ""
    private val clsNames = arrayOf("ボール", "黄ゴール", "青ゴール", "味方", "相手")
    private lateinit var prefs: android.content.SharedPreferences
    private lateinit var cover: View
    private var coverDownAt = 0L
    private var usbNote = "USB: 未接続"
    private var lastOut = ""
    private var uiTick = 0
    private val bauds = intArrayOf(115200, 460800, 921600, 1000000)
    private var baudIdx = 0

    private val permLauncher = registerForActivityResult(ActivityResultContracts.RequestPermission()) { ok ->
        if (ok) launchCamera() else status.text = "カメラ権限が必要です"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        prefs = getSharedPreferences("rcj", MODE_PRIVATE)
        Palette.load(prefs)
        Geometry.camHeightM = prefs.getInt("camH_cm", 15) / 100.0

        imu = Imu(this, tracker)
        usb = UsbOut(this) { msg -> usbNote = "USB: $msg"; Log.i(TAG, usbNote) }
        back = CamPipe(false, this, tracker, imu)
        front = CamPipe(true, this, tracker, imu)

        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.BLACK) }
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        fun cell(label: String): Triple<AspectFrame, PreviewView, Overlay> {
            val f = AspectFrame(this)
            val pv = PreviewView(this).apply {
                implementationMode = PreviewView.ImplementationMode.COMPATIBLE
                scaleType = PreviewView.ScaleType.FIT_CENTER
            }
            val ov = Overlay(this)
            val tag = TextView(this).apply { text = label; setTextColor(Color.WHITE); setPadding(8, 4, 8, 4); setBackgroundColor(0x88000000.toInt()) }
            f.addView(pv, ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT)
            f.addView(ov, ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT)
            f.addView(tag, FrameLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT, Gravity.START or Gravity.TOP))
            return Triple(f, pv, ov)
        }
        val (bf, bpv, bov) = cell("背面 (+Y 前方)")
        val (ff, fpv, fov) = cell("前面 (-Y 後方)")
        backView = bpv; frontView = fpv
        back.overlay = bov; front.overlay = fov
        row.addView(bf, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(ff, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(row)

        val btns = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        usbBtn = Button(this).apply { text = "USB接続"; setOnClickListener { usb.baud = bauds[baudIdx]; usb.connect() } }
        baudBtn = Button(this).apply {
            text = "${bauds[0]}bps"
            setOnClickListener {
                baudIdx = (baudIdx + 1) % bauds.size
                text = "${bauds[baudIdx]}bps"
            }
        }
        heightBtn = Button(this).apply {
            text = "高さ ${(Geometry.camHeightM * 100).toInt()}cm"
            setOnClickListener {
                val cm = ((Geometry.camHeightM * 100).toInt() % 40) + 1   // 1..40cmを巡回
                Geometry.camHeightM = cm / 100.0
                prefs.edit().putInt("camH_cm", cm).apply()
                text = "高さ ${cm}cm"
            }
        }
        val compBtn = Button(this).apply { text = "競技モード"; setOnClickListener { setCompetition(true) } }
        val hdgBtn = Button(this).apply { text = "方位0"; setOnClickListener { imu.resetHeading() } }
        for (b in listOf(usbBtn, baudBtn, heightBtn, hdgBtn, compBtn)) btns.addView(b, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(btns)

        val calRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        lateinit var calBtn: Button
        lateinit var clsBtn: Button
        calBtn = Button(this).apply {
            text = "校正OFF"
            setOnClickListener {
                calibrating = !calibrating
                text = if (calibrating) "校正ON" else "校正OFF"
                calTaps = 0
                calNote = if (calibrating) "校正: 対象を選んで、その色の物をタップ(複数回で範囲拡大)" else ""
            }
        }
        clsBtn = Button(this).apply {
            text = "対象:${clsNames[0]}"
            setOnClickListener {
                calCls = (calCls + 1) % clsNames.size
                calTaps = 0
                text = "対象:${clsNames[calCls]}"
            }
        }
        val resetBtn = Button(this).apply {
            text = "リセット"
            setOnClickListener {
                Palette.reset(calCls); Palette.save(prefs); calTaps = 0
                calNote = "${clsNames[calCls]}を初期値に戻しました: ${Palette.ranges[calCls]}"
            }
        }
        for (b in listOf(calBtn, clsBtn, resetBtn)) calRow.addView(b, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(calRow)

        // タップで色をサンプリング
        for (pipe in listOf(back, front)) {
            pipe.overlay.setOnTouchListener { v, ev ->
                if (calibrating && ev.action == android.view.MotionEvent.ACTION_UP) {
                    val nx = ev.x / v.width
                    val ny = ev.y / v.height
                    pipe.overlay.mark = floatArrayOf(nx, ny)
                    pipe.overlay.postInvalidate()
                    pipe.sampleCb = { hsv ->
                        val first = calTaps == 0
                        Palette.calibrate(calCls, hsv, first)
                        Palette.save(prefs)
                        calTaps++
                        calNote = String.format(
                            Locale.US, "校正[%s] タップ%d回 サンプルH%.0f S%.2f V%.2f → %s",
                            clsNames[calCls], calTaps, hsv[0], hsv[1], hsv[2], Palette.ranges[calCls],
                        )
                    }
                    pipe.sampleReq = floatArrayOf(nx, ny)
                }
                true
            }
        }

        status = TextView(this).apply {
            setTextColor(Color.rgb(200, 255, 200)); typeface = android.graphics.Typeface.MONOSPACE; textSize = 11f; setPadding(12, 8, 12, 8)
        }
        root.addView(ScrollView(this).apply { addView(status) }, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        cover = View(this).apply {
            setBackgroundColor(Color.BLACK)
            visibility = View.GONE
            // 長押し(2秒)で競技モード解除
            setOnTouchListener { _, ev ->
                when (ev.action) {
                    android.view.MotionEvent.ACTION_DOWN -> coverDownAt = SystemClock.uptimeMillis()
                    android.view.MotionEvent.ACTION_UP ->
                        if (SystemClock.uptimeMillis() - coverDownAt >= 2000) setCompetition(false)
                }
                true
            }
        }
        val frame = FrameLayout(this)
        frame.addView(root, ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT)
        frame.addView(cover, ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT)
        setContentView(frame)

        if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) launchCamera()
        else permLauncher.launch(Manifest.permission.CAMERA)
    }

    override fun onStart() {
        super.onStart()
        imu.start()
        ui.post(tick)
    }

    override fun onStop() {
        super.onStop()
        ui.removeCallbacks(altTick)
        imu.stop()
        ui.removeCallbacks(tick)
    }

    override fun onDestroy() {
        super.onDestroy()
        usb.release()
        camExec.shutdown()
    }

    /** 起動時: intent extra "competition"=true なら競技モードで開始 */
    private fun launchCamera() {
        if (intent.getBooleanExtra("competition", false)) setCompetition(true) else startCamera()
    }

    /** 競技モード: プレビュー無し・画面真っ黒・最低輝度・常時点灯。USBは自動接続 */
    private fun setCompetition(on: Boolean) {
        competition = on
        cover.visibility = if (on) View.VISIBLE else View.GONE
        val lp = window.attributes
        lp.screenBrightness = if (on) 0.01f else android.view.WindowManager.LayoutParams.BRIGHTNESS_OVERRIDE_NONE
        window.attributes = lp
        if (on) window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        else window.clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        startCamera()
        if (on && !usb.connected) { usb.baud = bauds[baudIdx]; usb.connect() }
    }

    private fun nmea(body: String): String {
        var cs = 0
        for (c in body) cs = cs xor c.code
        return String.format(Locale.US, "$%s*%02X\r\n", body, cs)
    }

    /** 30Hzで出力 + 5Hzで画面更新 */
    private val tick = object : Runnable {
        override fun run() {
            val now = SystemClock.elapsedRealtimeNanos()
            val tMs = now / 1_000_000
            val tracks = tracker.snapshot(now, 2, if (altActive) activeCam else -1)
            val sb = StringBuilder()
            val lin = imu.linAcc
            val g = imu.gyro
            sb.append(nmea(String.format(
                Locale.US, "IMU,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f",
                tMs, imu.headingDeg, imu.rollDeg, imu.pitchDeg,
                lin[0], lin[1], lin[2], g[0], g[1], g[2], imu.egoVx, imu.egoVy,
            )))
            for (t in tracks) {
                sb.append(nmea(String.format(
                    Locale.US, "OBJ,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    tMs, t.cls.code, t.id, t.cam,
                    t.x.toInt(), t.y.toInt(), t.vx.toInt(), t.vy.toInt(),
                    t.x500.toInt(), t.y500.toInt(), t.tcaMs, t.dcaMm, t.crossMs, t.crossXMm,
                    min(100, t.hits * 10),
                )))
            }
            lastOut = sb.toString()
            usb.send(lastOut)
            if (!competition) Log.d(TAG, lastOut.trimEnd())
            if (uiTick % 90 == 0) Log.i(TAG, String.format(Locale.US, "fps back=%.1f front=%.1f %s", back.fps, front.fps, camNote))

            if (++uiTick % 6 == 0 && !competition) {
                status.text = buildString {
                    appendLine("$camNote  背面${String.format(Locale.US, "%.0f", back.fps)}fps 前面${String.format(Locale.US, "%.0f", front.fps)}fps")
                    appendLine(usbNote)
                    if (calNote.isNotEmpty()) appendLine(calNote)
                    if (calibrating) for (i in clsNames.indices) appendLine("  ${clsNames[i]}: ${Palette.ranges[i]}")
                    appendLine(String.format(Locale.US, "方位%.0f° roll%.0f° pitch%.0f° 自機速度(%.2f,%.2f)m/s", imu.headingDeg, imu.rollDeg, imu.pitchDeg, imu.egoVx, imu.egoVy))
                    appendLine("--- 追跡 (x,y mm / v mm/s / 0.5s後 / 最接近) ---")
                    for (t in tracks) appendLine(String.format(
                        Locale.US, "%s#%d c%d (%5d,%5d) v(%4d,%4d) →(%5d,%5d) tca%dms d%dmm",
                        t.cls.code, t.id, t.cam, t.x.toInt(), t.y.toInt(), t.vx.toInt(), t.vy.toInt(),
                        t.x500.toInt(), t.y500.toInt(), t.tcaMs, t.dcaMm,
                    ))
                    appendLine("--- USB出力 ---")
                    append(lastOut.lines().take(6).joinToString("\n"))
                }
            }
            ui.postDelayed(this, 33)
        }
    }

    @androidx.annotation.OptIn(ExperimentalCamera2Interop::class)
    private fun group(pipe: CamPipe, pv: PreviewView, withPreview: Boolean): UseCaseGroup {
        val sel = ResolutionSelector.Builder()
            .setResolutionStrategy(ResolutionStrategy(Size(320, 240), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
            .build()
        val ab = ImageAnalysis.Builder()
            .setResolutionSelector(sel)
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
        // 30fps固定を要求(暗いと自動露出でfpsが落ちるのを抑える)
        androidx.camera.camera2.interop.Camera2Interop.Extender(ab)
            .setCaptureRequestOption(android.hardware.camera2.CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, android.util.Range(30, 30))
        val analysis = ab.build()
        analysis.setAnalyzer(camExec) { pipe.analyze(it) }
        val g = UseCaseGroup.Builder().addUseCase(analysis)
        if (withPreview) {
            g.addUseCase(Preview.Builder().setResolutionSelector(sel).build().also { it.setSurfaceProvider(pv.surfaceProvider) })
        }
        return g.build()
    }

    /** 前後どちらか一方だけをバインド。交互切替で両方を使う */
    @androidx.annotation.OptIn(ExperimentalCamera2Interop::class)
    private fun bindAlt(useFront: Boolean) {
        val provider = cameraProvider ?: return
        provider.unbindAll()
        val pipe = if (useFront) front else back
        val pv = if (useFront) frontView else backView
        val sel = if (useFront) CameraSelector.DEFAULT_FRONT_CAMERA else CameraSelector.DEFAULT_BACK_CAMERA
        try {
            val cam = provider.bindToLifecycle(this, sel, group(pipe, pv, !competition))
            pipe.cameraId = Camera2CameraInfo.from(cam.cameraInfo).cameraId
            activeCam = if (useFront) 1 else 0
        } catch (e: Exception) {
            Log.e(TAG, "bindAlt failed", e)
        }
    }

    private val altTick = object : Runnable {
        override fun run() {
            if (!altActive) return
            bindAlt(activeCam == 0)
            ui.postDelayed(this, if (activeCam == 0) ALT_BACK_MS else ALT_FRONT_MS)
        }
    }

    @androidx.annotation.OptIn(ExperimentalCamera2Interop::class)
    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            val provider = future.get()
            provider.unbindAll()
            ui.removeCallbacks(altTick)
            altActive = false
            val concurrent = provider.availableConcurrentCameraInfos.any { infos ->
                infos.any { it.lensFacing == CameraSelector.LENS_FACING_BACK } &&
                    infos.any { it.lensFacing == CameraSelector.LENS_FACING_FRONT }
            }
            try {
                if (concurrent) {
                    val cam: ConcurrentCamera = provider.bindToLifecycle(listOf(
                        ConcurrentCamera.SingleCameraConfig(CameraSelector.DEFAULT_BACK_CAMERA, group(back, backView, !competition), this),
                        ConcurrentCamera.SingleCameraConfig(CameraSelector.DEFAULT_FRONT_CAMERA, group(front, frontView, !competition), this),
                    ))
                    back.cameraId = Camera2CameraInfo.from(cam.cameras[0].cameraInfo).cameraId
                    front.cameraId = Camera2CameraInfo.from(cam.cameras[1].cameraInfo).cameraId
                    camNote = "前後同時"
                } else {
                    cameraProvider = provider
                    altActive = true
                    camNote = "前後交互(同時利用非対応)"
                    bindAlt(false)
                    ui.postDelayed(altTick, ALT_BACK_MS)
                }
            } catch (e: Exception) {
                Log.e(TAG, "bind failed", e)
                camNote = "カメラ起動失敗: ${e.message}"
            }
            Log.i(TAG, "camera: $camNote")
        }, ContextCompat.getMainExecutor(this))
    }
}
