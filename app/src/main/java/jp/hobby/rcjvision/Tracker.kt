package jp.hobby.rcjvision

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** 1つの追跡対象。位置mm, 速度mm/s (ロボット座標系) */
class Track(val id: Int, val cls: Cls, var x: Double, var y: Double, var tNs: Long, var cam: Int) {
    var vx = 0.0
    var vy = 0.0
    var hits = 1
}

/** 出力用スナップショット(現在時刻へ外挿済み) + 進路予測 */
class TrackOut(
    val id: Int, val cls: Cls, val cam: Int, val hits: Int,
    val x: Double, val y: Double, val vx: Double, val vy: Double,
    val x500: Double, val y500: Double,   // 0.5秒後の予測位置
    val tcaMs: Int, val dcaMm: Int,       // 最接近までの時間と最接近距離(スマホ原点)
    val crossMs: Int, val crossXMm: Int,  // Y=0ラインを横切るまでの時間と、その時のX (横切らないなら-1)
)

class Tracker {
    private val tracks = ArrayList<Track>()
    private var nextId = 1

    private val alpha = 0.6
    private val beta = 0.25

    private fun gate(cls: Cls) = when (cls) {
        Cls.BALL -> 600.0
        Cls.GOAL_Y, Cls.GOAL_B -> 1500.0
        else -> 800.0
    }

    private fun ttl(cls: Cls) = if (cls == Cls.BALL) 0.6 else 1.0

    /** 自機が反時計回りにdYaw[rad]回頭した → 静止物は座標系上で逆回転して見える */
    @Synchronized
    fun rotate(dYaw: Double) {
        val c = cos(-dYaw)
        val s = sin(-dYaw)
        for (t in tracks) {
            val nx = t.x * c - t.y * s
            val ny = t.x * s + t.y * c
            t.x = nx; t.y = ny
            val nvx = t.vx * c - t.vy * s
            val nvy = t.vx * s + t.vy * c
            t.vx = nvx; t.vy = nvy
        }
    }

    @Synchronized
    fun update(cls: Cls, cam: Int, xMm: Double, yMm: Double, nowNs: Long) {
        var best: Track? = null
        var bestD = gate(cls)
        for (t in tracks) {
            if (t.cls != cls) continue
            val dt = (nowNs - t.tNs) / 1e9
            val d = hypot(xMm - (t.x + t.vx * dt), yMm - (t.y + t.vy * dt))
            if (d < bestD) { bestD = d; best = t }
        }
        if (best == null) {
            tracks += Track(nextId++, cls, xMm, yMm, nowNs, cam)
            return
        }
        val dt = ((nowNs - best.tNs) / 1e9).coerceIn(0.005, 0.5)
        val px = best.x + best.vx * dt
        val py = best.y + best.vy * dt
        val rx = xMm - px
        val ry = yMm - py
        best.x = px + alpha * rx
        best.y = py + alpha * ry
        best.vx += beta * rx / dt
        best.vy += beta * ry / dt
        best.tNs = nowNs
        best.cam = cam
        best.hits++
    }

    @Synchronized
    fun snapshot(nowNs: Long, minHits: Int = 2, activeCam: Int = -1): List<TrackOut> {
        // 交互切替中は、いま見ていない側のカメラで最後に見えた物は寿命を3倍にして残す
        tracks.removeAll {
            val k = if (activeCam >= 0 && it.cam != activeCam) 3.0 else 1.0
            (nowNs - it.tNs) / 1e9 > ttl(it.cls) * k
        }
        val out = ArrayList<TrackOut>()
        for (t in tracks) {
            if (t.hits < minHits) continue
            val dt = (nowNs - t.tNs) / 1e9
            val x = t.x + t.vx * dt
            val y = t.y + t.vy * dt
            val sp2 = t.vx * t.vx + t.vy * t.vy
            var tca = 0.0
            if (sp2 > 100.0) tca = (-(x * t.vx + y * t.vy) / sp2).coerceAtLeast(0.0).coerceAtMost(5.0)
            val dca = hypot(x + t.vx * tca, y + t.vy * tca)
            var cross = -1
            var crossX = -1
            if (y * t.vy < 0 && kotlin.math.abs(t.vy) > 50.0) {
                val tc = -y / t.vy
                if (tc < 5.0) { cross = (tc * 1000).toInt(); crossX = (x + t.vx * tc).toInt() }
            }
            out += TrackOut(
                t.id, t.cls, t.cam, t.hits, x, y, t.vx, t.vy,
                x + t.vx * 0.5, y + t.vy * 0.5,
                (tca * 1000).toInt(), dca.toInt(), cross, crossX,
            )
        }
        return out
    }
}
