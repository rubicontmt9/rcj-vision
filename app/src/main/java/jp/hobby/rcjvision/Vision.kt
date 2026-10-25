package jp.hobby.rcjvision

import android.graphics.Color
import androidx.camera.core.ImageProxy
import kotlin.math.max
import kotlin.math.min

/** 検出クラス。code はUSB出力で使う1文字 */
enum class Cls(val code: String, val minCells: Int, val maxKeep: Int) {
    BALL("B", 3, 1),      // オレンジのボール
    GOAL_Y("Y", 60, 2),   // 黄ゴール
    GOAL_B("U", 60, 2),   // 青ゴール
    ALLY("A", 8, 3),      // 味方ロボットのマーカー色(マゼンタ)
    OPP("O", 8, 3),       // 相手ロボットのマーカー色(シアン)
}

class Blob(
    val cls: Cls,
    val cx: Float, val cy: Float,
    val x0: Float, val y0: Float, val x1: Float, val y1: Float,
    val cells: Int, val fill: Float,
) {
    var xMm = Float.NaN
    var yMm = Float.NaN
}

/** 画素座標は全て「正立画像(w×h)」基準 */
class Detection(val w: Int, val h: Int, val blobs: List<Blob>)

/** HSV範囲。hLo>hHi のときは色相が0度をまたぐ範囲(赤など) */
class HsvRange(var hLo: Float, var hHi: Float, var sMin: Float, var vMin: Float) {
    fun contains(h: Float, s: Float, v: Float) =
        s >= sMin && v >= vMin && (if (hLo <= hHi) h in hLo..hHi else h >= hLo || h <= hHi)

    fun copy() = HsvRange(hLo, hHi, sMin, vMin)
    override fun toString() = "H%.0f-%.0f S>=%.2f V>=%.2f".format(hLo, hHi, sMin, vMin)
}

/** 色の判定範囲。現場でタップ校正して SharedPreferences に保存する */
object Palette {
    private val defaults = arrayOf(
        HsvRange(8f, 30f, 0.55f, 0.45f),     // BALL 橙
        HsvRange(42f, 68f, 0.45f, 0.45f),    // GOAL_Y 黄
        HsvRange(200f, 255f, 0.50f, 0.25f),  // GOAL_B 青
        HsvRange(285f, 335f, 0.40f, 0.30f),  // ALLY マゼンタ
        HsvRange(165f, 199f, 0.50f, 0.30f),  // OPP シアン
    )
    val ranges = Array(defaults.size) { defaults[it].copy() }

    /** RGB各5bit量子化 → クラス+1 (0=該当なし)。毎画素のHSV変換を避けるための早見表 */
    @Volatile var lut = build()

    fun classify(h: Float, s: Float, v: Float): Int {
        for (i in ranges.indices) if (ranges[i].contains(h, s, v)) return i
        return -1
    }

    private fun build(): ByteArray {
        val t = ByteArray(32768)
        val hsv = FloatArray(3)
        for (i in 0 until 32768) {
            val r = ((i shr 10) and 31) * 8 + 4
            val g = ((i shr 5) and 31) * 8 + 4
            val b = (i and 31) * 8 + 4
            Color.RGBToHSV(r, g, b, hsv)
            t[i] = (classify(hsv[0], hsv[1], hsv[2]) + 1).toByte()
        }
        return t
    }

    fun rebuild() { lut = build() }

    private fun wrap(h: Float): Float { var x = h % 360f; if (x < 0) x += 360f; return x }

    /** タップで得たHSVからクラスの範囲を設定。first=trueなら置き換え、falseなら範囲を広げる */
    fun calibrate(cls: Int, hsv: FloatArray, first: Boolean) {
        val h = hsv[0]; val s = hsv[1]; val v = hsv[2]
        val r = ranges[cls]
        val sNew = maxOf(0.25f, s * 0.6f)
        val vNew = maxOf(0.15f, v * 0.5f)
        if (first) {
            r.hLo = wrap(h - 10f); r.hHi = wrap(h + 10f); r.sMin = sNew; r.vMin = vNew
        } else {
            if (!(if (r.hLo <= r.hHi) h in r.hLo..r.hHi else h >= r.hLo || h <= r.hHi)) {
                val dLo = wrap(r.hLo - h)   // 下側に外れている量
                val dHi = wrap(h - r.hHi)   // 上側に外れている量
                if (dLo <= dHi) r.hLo = wrap(h - 6f) else r.hHi = wrap(h + 6f)
            }
            r.sMin = minOf(r.sMin, sNew); r.vMin = minOf(r.vMin, vNew)
        }
        rebuild()
    }

    fun reset(cls: Int) { ranges[cls] = defaults[cls].copy(); rebuild() }

    fun save(prefs: android.content.SharedPreferences) {
        val e = prefs.edit()
        for (i in ranges.indices) e.putString("pal$i", "${ranges[i].hLo},${ranges[i].hHi},${ranges[i].sMin},${ranges[i].vMin}")
        e.apply()
    }

    fun load(prefs: android.content.SharedPreferences) {
        for (i in ranges.indices) {
            val p = prefs.getString("pal$i", null)?.split(",")?.mapNotNull { it.toFloatOrNull() } ?: continue
            if (p.size == 4) ranges[i] = HsvRange(p[0], p[1], p[2], p[3])
        }
        rebuild()
    }
}

class Detector {
    private var mask = ByteArray(0)
    private var seen = BooleanArray(0)
    private var stack = IntArray(0)

    fun detect(img: ImageProxy): Detection {
        val w = img.width
        val h = img.height
        val rot = img.imageInfo.rotationDegrees
        val step = max(2, w / 160)
        val gw = w / step
        val gh = h / step
        val n = gw * gh
        if (mask.size != n) {
            mask = ByteArray(n); seen = BooleanArray(n); stack = IntArray(n)
        }
        java.util.Arrays.fill(seen, false)

        // YUV_420_888 を直接サンプリング(RGB変換済み画像を作らない)
        val py = img.planes[0]; val pu = img.planes[1]; val pv = img.planes[2]
        val yb = py.buffer; val ub = pu.buffer; val vb = pv.buffer
        val yrs = py.rowStride
        val crs = pu.rowStride; val cps = pu.pixelStride
        val lut = Palette.lut
        for (gy in 0 until gh) {
            val y = gy * step + step / 2
            val yRow = y * yrs
            val cRow = (y shr 1) * crs
            for (gx in 0 until gw) {
                val x = gx * step + step / 2
                val yy = yb.get(yRow + x).toInt() and 0xff
                val ci = cRow + (x shr 1) * cps
                val u = (ub.get(ci).toInt() and 0xff) - 128
                val v = (vb.get(ci).toInt() and 0xff) - 128
                var r = yy + ((359 * v) shr 8)
                var g = yy - ((88 * u + 183 * v) shr 8)
                var b = yy + ((454 * u) shr 8)
                if (r < 0) r = 0 else if (r > 255) r = 255
                if (g < 0) g = 0 else if (g > 255) g = 255
                if (b < 0) b = 0 else if (b > 255) b = 255
                mask[gy * gw + gx] = lut[((r shr 3) shl 10) or ((g shr 3) shl 5) or (b shr 3)]
            }
        }

        val uw = if (rot % 180 == 0) w else h
        val uh = if (rot % 180 == 0) h else w
        val found = ArrayList<Blob>()

        for (start in 0 until n) {
            val m = mask[start].toInt()
            if (m == 0 || seen[start]) continue
            val cls = Cls.values()[m - 1]
            var sp = 0
            stack[sp++] = start
            seen[start] = true
            var cnt = 0
            var minX = gw; var maxX = 0; var minY = gh; var maxY = 0
            var sumX = 0L; var sumY = 0L
            while (sp > 0) {
                val p = stack[--sp]
                val px = p % gw
                val py = p / gw
                cnt++
                sumX += px; sumY += py
                if (px < minX) minX = px
                if (px > maxX) maxX = px
                if (py < minY) minY = py
                if (py > maxY) maxY = py
                if (px > 0 && visit(p - 1, m)) stack[sp++] = p - 1
                if (px < gw - 1 && visit(p + 1, m)) stack[sp++] = p + 1
                if (py > 0 && visit(p - gw, m)) stack[sp++] = p - gw
                if (py < gh - 1 && visit(p + gw, m)) stack[sp++] = p + gw
            }
            if (cnt < cls.minCells) continue
            val bw = maxX - minX + 1
            val bh = maxY - minY + 1
            val fill = cnt.toFloat() / (bw * bh)
            if (cls == Cls.BALL) {
                val aspect = bw.toFloat() / bh
                if (fill < 0.45f || aspect < 0.5f || aspect > 2f) continue
            }
            // グリッド → センサー画素 → 正立画素
            val sx0 = minX * step.toFloat(); val sx1 = (maxX + 1) * step.toFloat()
            val sy0 = minY * step.toFloat(); val sy1 = (maxY + 1) * step.toFloat()
            val scx = (sumX.toFloat() / cnt + 0.5f) * step
            val scy = (sumY.toFloat() / cnt + 0.5f) * step
            val a = upright(sx0, sy0, w, h, rot)
            val b = upright(sx1, sy1, w, h, rot)
            val c = upright(scx, scy, w, h, rot)
            found += Blob(
                cls, c[0], c[1],
                min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]),
                cnt, fill,
            )
        }

        val keep = ArrayList<Blob>()
        for (cls in Cls.values()) {
            found.filter { it.cls == cls }.sortedByDescending { it.cells }.take(cls.maxKeep).forEach { keep += it }
        }
        return Detection(uw, uh, keep)
    }

    /** 正立画像の正規化座標(nx,ny)付近5x5画素の平均HSV [h,s,v]。校正用 */
    fun sample(img: ImageProxy, nx: Float, ny: Float): FloatArray {
        val w = img.width; val h = img.height
        val rot = img.imageInfo.rotationDegrees
        val uw = if (rot % 180 == 0) w else h
        val uh = if (rot % 180 == 0) h else w
        val u = nx * uw; val v = ny * uh
        val (cx, cy) = when (rot) {
            90 -> Pair(v, h - u)
            180 -> Pair(w - u, h - v)
            270 -> Pair(w - v, u)
            else -> Pair(u, v)
        }
        val py = img.planes[0]; val pu = img.planes[1]; val pv = img.planes[2]
        var sr = 0; var sg = 0; var sb = 0; var n = 0
        for (dy in -2..2) for (dx in -2..2) {
            val x = (cx.toInt() + dx).coerceIn(0, w - 1)
            val y = (cy.toInt() + dy).coerceIn(0, h - 1)
            val yy = py.buffer.get(y * py.rowStride + x).toInt() and 0xff
            val ci = (y shr 1) * pu.rowStride + (x shr 1) * pu.pixelStride
            val uu = (pu.buffer.get(ci).toInt() and 0xff) - 128
            val vv = (pv.buffer.get(ci).toInt() and 0xff) - 128
            sr += (yy + ((359 * vv) shr 8)).coerceIn(0, 255)
            sg += (yy - ((88 * uu + 183 * vv) shr 8)).coerceIn(0, 255)
            sb += (yy + ((454 * uu) shr 8)).coerceIn(0, 255)
            n++
        }
        val out = FloatArray(3)
        Color.RGBToHSV(sr / n, sg / n, sb / n, out)
        return out
    }

    /** 未訪問かつ同クラスなら訪問済みにしてtrue */
    private fun visit(idx: Int, m: Int): Boolean {
        if (seen[idx] || mask[idx].toInt() != m) return false
        seen[idx] = true
        return true
    }

    /** センサー画素(x,y) → 正立画像の画素。rot は「時計回りに回すと正立」になる角度 */
    private fun upright(x: Float, y: Float, w: Int, h: Int, rot: Int): FloatArray = when (rot) {
        90 -> floatArrayOf(h - y, x)
        180 -> floatArrayOf(w - x, h - y)
        270 -> floatArrayOf(y, w - x)
        else -> floatArrayOf(x, y)
    }
}
