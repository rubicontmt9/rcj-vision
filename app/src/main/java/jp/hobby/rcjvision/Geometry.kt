package jp.hobby.rcjvision

import kotlin.math.sqrt

/**
 * ロボット座標系: 原点=スマホ(背面カメラ位置), +Y=背面カメラの向き(前方), +X=その右, 単位mm。
 * 前面カメラで見た物体は Y<0 側(後方)に出る。
 * 重力ベクトル(加速度センサー)で床面を決め、画素の視線を床面(または指定高さの面)と交差させて座標を出す。
 */
object Geometry {
    @Volatile var camHeightM = 0.15      // カメラの床からの高さ
    @Volatile var markerHeightM = 0.12   // 味方/相手マーカー中心の高さ

    private fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    /** 機体の前方F・右Rを端末座標系で返す。スマホが水平すぎて定義できない時はnull */
    fun axes(up: DoubleArray): Array<DoubleArray>? {
        val f0 = doubleArrayOf(0.0, 0.0, -1.0)           // 背面カメラの光軸(端末座標)
        val k = dot(f0, up)
        val f = doubleArrayOf(f0[0] - k * up[0], f0[1] - k * up[1], f0[2] - k * up[2])
        val n = sqrt(dot(f, f))
        if (n < 0.2) return null
        for (i in 0..2) f[i] /= n
        val r = doubleArrayOf(
            f[1] * up[2] - f[2] * up[1],
            f[2] * up[0] - f[0] * up[2],
            f[0] * up[1] - f[1] * up[0],
        )
        return arrayOf(f, r)
    }

    /**
     * 正立画像の画素(u,v) → ロボット座標[x_mm, y_mm]。
     * planeZ は物体の高さ(m)。見つからなければnull。
     */
    fun project(
        front: Boolean, u: Double, v: Double, w: Int, h: Int,
        focalPx: Double, up: DoubleArray, planeZ: Double,
    ): DoubleArray? {
        val right = (u - w / 2.0) / focalPx
        val upv = -(v - h / 2.0) / focalPx
        val d = if (front) doubleArrayOf(-right, upv, 1.0) else doubleArrayOf(right, upv, -1.0)
        val dn = dot(d, up)
        if (dn > -1e-3) return null
        val t = (planeZ - camHeightM) / dn
        if (t <= 0) return null
        val p = doubleArrayOf(d[0] * t, d[1] * t, d[2] * t)
        val ax = axes(up) ?: return null
        return doubleArrayOf(dot(p, ax[1]) * 1000.0, dot(p, ax[0]) * 1000.0)
    }
}
