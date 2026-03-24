"""
auto_port.py — PlatformIO pre-upload script
Scans USB serial ports by VID/PID and sets UPLOAD_PORT and MONITOR_PORT
so flashing works regardless of what COM number Windows assigns.
"""
Import("env")
import serial.tools.list_ports

# Ordered by preference — first match wins
KNOWN = [
    (0x303A, 0x1001, "ESP32-S3 native USB CDC"),
    (0x303A, 0x1000, "ESP32-S3 JTAG/bootloader"),
    (0x10C4, 0xEA60, "CP2102N UART bridge"),
    (0x1A86, 0x7523, "CH340 UART bridge"),
    (0x0403, 0x6001, "FTDI UART bridge"),
]

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for vid, pid, label in KNOWN:
        for p in ports:
            if p.vid == vid and p.pid == pid:
                return p.device, label
    return None, None

port, label = find_port()
if port:
    print(f"\nauto_port: found {label} on {port}")
    env.Replace(UPLOAD_PORT=port)
    env.Replace(MONITOR_PORT=port)
else:
    print("\nauto_port: no known ESP32 device found — falling back to PlatformIO default")
