package jp.hobby.rcjvision

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * 加速度/ジャイロ統合。
 * - up: 重力方向(端末座標)。床面と機体座標系の決定に使う
 * - ジャイロの上方向成分で自機の回頭を積分し、追跡中の物体座標を回す
 * - 直線加速度を機体座標へ変換し、減衰付き積分で自機速度の目安を出す(ドリフトするので参考値)
 */
class Imu(ctx: Context, private val tracker: Tracker) : SensorEventListener {
    private val sm = ctx.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    @Volatile var up = doubleArrayOf(0.0, 1.0, 0.0)
    @Volatile var gyro = DoubleArray(3)
    @Volatile var linAcc = DoubleArray(3)
    @Volatile var headingDeg = 0.0
    @Volatile var rollDeg = 0.0
    @Volatile var pitchDeg = 0.0
    @Volatile var egoVx = 0.0   // m/s 機体座標
    @Volatile var egoVy = 0.0

    private var lastGyroNs = 0L
    private var lastLinNs = 0L
    private var yaw0 = Double.NaN
    private val rot = FloatArray(9)
    private val ori = FloatArray(3)
    private var hasGravity = false

    fun start() {
        val grav = sm.getDefaultSensor(Sensor.TYPE_GRAVITY)
        hasGravity = grav != null
        val list = listOf(
            grav ?: sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
            sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
            sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
            sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR),
        )
        for (s in list) if (s != null) sm.registerListener(this, s, 10_000)
    }

    fun stop() = sm.unregisterListener(this)

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_GRAVITY, Sensor.TYPE_ACCELEROMETER -> {
                val v = doubleArrayOf(e.values[0].toDouble(), e.values[1].toDouble(), e.values[2].toDouble())
                val n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
                if (n < 1e-3) return
                val k = if (hasGravity) 1.0 else 0.1   // 加速度センサーのみの場合はローパス
                val o = up
                val m = DoubleArray(3) { o[it] * (1 - k) + v[it] / n * k }
                val mn = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2])
                up = DoubleArray(3) { m[it] / mn }
            }
            Sensor.TYPE_GYROSCOPE -> {
                val g = doubleArrayOf(e.values[0].toDouble(), e.values[1].toDouble(), e.values[2].toDouble())
                gyro = g
                if (lastGyroNs != 0L) {
                    val dt = (e.timestamp - lastGyroNs) / 1e9
                    val u = up
                    val wUp = g[0] * u[0] + g[1] * u[1] + g[2] * u[2]
                    if (dt in 0.0..0.2) tracker.rotate(wUp * dt)
                }
                lastGyroNs = e.timestamp
            }
            Sensor.TYPE_LINEAR_ACCELERATION -> {
                val a = doubleArrayOf(e.values[0].toDouble(), e.values[1].toDouble(), e.values[2].toDouble())
                linAcc = a
                if (lastLinNs != 0L) {
                    val dt = (e.timestamp - lastLinNs) / 1e9
                    val ax = Geometry.axes(up)
                    if (ax != null && dt in 0.0..0.2) {
                        val ar = a[0] * ax[1][0] + a[1] * ax[1][1] + a[2] * ax[1][2]
                        val af = a[0] * ax[0][0] + a[1] * ax[0][1] + a[2] * ax[0][2]
                        val decay = exp(-dt / 1.0)
                        egoVx = egoVx * decay + ar * dt
                        egoVy = egoVy * decay + af * dt
                    }
                }
                lastLinNs = e.timestamp
            }
            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                SensorManager.getRotationMatrixFromVector(rot, e.values)
                SensorManager.getOrientation(rot, ori)
                val yaw = Math.toDegrees(ori[0].toDouble())
                if (yaw0.isNaN()) yaw0 = yaw
                var d = yaw - yaw0
                while (d > 180) d -= 360
                while (d < -180) d += 360
                headingDeg = d
                pitchDeg = Math.toDegrees(ori[1].toDouble())
                rollDeg = Math.toDegrees(ori[2].toDouble())
            }
        }
    }

    /** 機体の前方を基準に、現在の向きを0度にリセット */
    fun resetHeading() { yaw0 = Double.NaN }
}
