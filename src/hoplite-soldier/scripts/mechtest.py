#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import rospy
from limo_base.msg import LimoStatus
import serial
import struct
import time

# --- Constants (match C++ MSG_CTRL_MODE_CONFIG_ID) ---
MSG_CTRL_MODE_CONFIG_ID = 0x0103   # < **You must set this to the actual ID** from your limo_driver.h
FRAME_HEADER        = 0x55
FRAME_LEN           = 0x0e

# Serial port parameters
PORT_NAME  = "/dev/ttyTHS1"         # adjust as needed
BAUDRATE   = 460800

# Globals for callback
last_status = None

def status_callback(msg):
    global last_status
    last_status = msg

def build_frame(mode_byte):
    # data[0]=0x01 (enable), data[1]=0x00, data[2]=mode_byte, rest=0
    data = [0x01, 0x00, mode_byte] + [0]*5
    checksum = sum(data) & 0xFF
    data.append(0x00)
    data.append(checksum)
    # pack: header, length, id_hi, id_lo, data[0..7], count(0), checksum
    frame = struct.pack(">BBH8B B B",
                        FRAME_HEADER,
                        FRAME_LEN,
                        MSG_CTRL_MODE_CONFIG_ID,
                        *data)
    return frame

def main():
    global last_status
    rospy.init_node("mecanum_mode_finder", anonymous=True)
    rospy.Subscriber("/limo_status", LimoStatus, status_callback)  # :contentReference[oaicite:2]{index=2}

    # open serial port
    ser = serial.Serial(PORT_NAME, BAUDRATE, timeout=0.1)          # :contentReference[oaicite:3]{index=3}
    time.sleep(0.5)

    results = {}
    for mode in range(4):
        last_status = None
        frame = build_frame(mode)
        ser.write(frame)                                           # :contentReference[oaicite:4]{index=4}
        rospy.loginfo("Tried data[2]=0x%02X, waiting for status…" % mode)
        # wait for next status message
        t0 = time.time()
        while time.time() - t0 < 1.0 and not rospy.is_shutdown():
            rospy.sleep(0.05)
            if last_status:
                results[mode] = last_status.motion_mode
                rospy.loginfo("? got motion_mode = %d" % last_status.motion_mode)
                break
        else:
            results[mode] = None
            rospy.logwarn("? no status reply for mode 0x%02X" % mode)

    # print summary
    print("\nMode byte ? motion_mode mapping:")
    for mode, mval in results.items():
        print("0x%02X ? %s" % (mode, str(mval)))

    # write to file
    with open("mode_scan_results.csv", "w") as f:
        f.write("mode_byte,motion_mode\n")
        for mode, mval in results.items():
            f.write("%d,%s\n" % (mode, str(mval)))

    ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
