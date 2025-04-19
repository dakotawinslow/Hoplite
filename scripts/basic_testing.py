#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time

limo = limo.LIMO()
limo.EnableCommand()  # 使能控制s
num = 10
while num > 0:
    limo.SetMotionCommand(linear_vel=1, steering_angle=20)
    time.sleep(0.3)
    num -= 1

quit()
