package jp.hobby.rcjvision

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicInteger

/**
 * スマホ(USBホスト) → マイコンへのシリアル出力。
 * CH340/CP210x/FTDI など既知のUSBシリアルIC、およびRP2040/STM32/ESP32-Sx/CH32 等のネイティブCDC-ACMに対応。
 */
class UsbOut(private val ctx: Context, private val onState: (String) -> Unit) {
    private val mgr = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
    private val exec = Executors.newSingleThreadExecutor()
    private val pending = AtomicInteger(0)
    private var port: UsbSerialPort? = null
    private var registered = false
    var baud = 115200
    val connected get() = port != null

    private val action = "jp.hobby.rcjvision.USB_PERMISSION"

    private val rx = object : BroadcastReceiver() {
        override fun onReceive(c: Context, i: Intent) {
            if (i.action != action) return
            val dev = i.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
            if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false) && dev != null) open(dev)
            else onState("USB権限が拒否されました")
        }
    }

    private fun findDriver(): UsbSerialDriver? {
        val def = UsbSerialProber.getDefaultProber()
        for (d in mgr.deviceList.values) {
            def.probeDevice(d)?.let { return it }
            val cdc = (0 until d.interfaceCount).any { d.getInterface(it).interfaceClass in intArrayOf(2, 10) }
            if (cdc) return CdcAcmSerialDriver(d)
        }
        return null
    }

    fun connect() {
        close()
        val driver = findDriver()
        if (driver == null) {
            onState("USBシリアル機器が見つかりません(OTGでマイコンを接続してください)")
            return
        }
        if (!registered) {
            ctx.registerReceiver(rx, IntentFilter(action), Context.RECEIVER_NOT_EXPORTED)
            registered = true
        }
        if (mgr.hasPermission(driver.device)) {
            open(driver.device)
        } else {
            val pi = PendingIntent.getBroadcast(
                ctx, 0, Intent(action).setPackage(ctx.packageName), PendingIntent.FLAG_MUTABLE,
            )
            mgr.requestPermission(driver.device, pi)
            onState("USB権限を許可してください")
        }
    }

    private fun open(dev: UsbDevice) {
        val driver = findDriver()?.takeIf { it.device.deviceId == dev.deviceId } ?: return
        val conn = mgr.openDevice(dev)
        if (conn == null) { onState("USBを開けません"); return }
        try {
            val p = driver.ports[0]
            p.open(conn)
            p.setParameters(baud, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            // ESP32-S3 内蔵USB(VID 303A)はDTR/RTSの操作でリセットやブートローダ突入が起きるため一切触らない
            if (dev.vendorId != 0x303A) {
                p.dtr = true
                p.rts = false
            }
            port = p
            val name = dev.productName?.filter { it.code in 32..126 }?.ifBlank { null }
            onState("USB接続: ${name ?: "VID%04X:PID%04X".format(dev.vendorId, dev.productId)} @$baud")
        } catch (e: Exception) {
            onState("USBエラー: ${e.message}")
            close()
        }
    }

    fun send(text: String) {
        val p = port ?: return
        if (pending.get() > 3) return   // 詰まったら古い出力は捨てる
        pending.incrementAndGet()
        exec.execute {
            try {
                p.write(text.toByteArray(Charsets.US_ASCII), 100)
            } catch (e: Exception) {
                port = null
                onState("USB切断: ${e.message}")
            } finally {
                pending.decrementAndGet()
            }
        }
    }

    fun close() {
        try { port?.close() } catch (_: Exception) {}
        port = null
    }

    fun release() {
        close()
        if (registered) { ctx.unregisterReceiver(rx); registered = false }
    }
}
