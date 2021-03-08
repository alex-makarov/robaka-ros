#!/usr/bin/env python
import rospy
import time
import os
from std_msgs.msg import String, Float64

class State:
    voltage       = 0.0
    left_current  = 0.0
    right_current = 0.0
    last_update   = 0.0

state = State()

def voltage_callback(data):
    state.last_update = time.time()
    state.voltage = data.data

def left_current_callback(data):
    state.left_current = data.data
    
def right_current_callback(data):
    state.right_current = data.data

def send_update(pub):
    # It's a 2x16 screen
    if time.time() - state.last_update < 1.0:
        current = 1000*(state.left_current + state.right_current)/2.0
        message = '{voltage:.1f}V {current:.0f}mA'.format(voltage=state.voltage, current=current)
    else:
        message = "Engine OFF"

    nano_current = float(os.popen("cat /sys/bus/i2c/devices/6-0040/iio_device/in_power0_input").read())
    message += " {c:.0f}mA\n".format(c=nano_current)

    la = os.getloadavg()[0]
    cpu_temp = float(os.popen("cat /sys/devices/virtual/thermal/thermal_zone1/temp").read())/1000.0
    gpu_temp = float(os.popen("cat /sys/devices/virtual/thermal/thermal_zone2/temp").read())/1000.0
    message += "LA:{la:.1f} C:{cpu_temp:.0f} G:{gpu_temp:.0f}".format(la=la, cpu_temp=cpu_temp, gpu_temp=gpu_temp)

#    rospy.loginfo(message)
    pub.publish(message)
    
def main():
    rospy.init_node('display')

    rospy.Subscriber("/hoverboard/battery_voltage", Float64, voltage_callback)
    rospy.Subscriber("/hoverboard/left_wheel/current", Float64, left_current_callback)
    rospy.Subscriber("/hoverboard/right_wheel/current", Float64, right_current_callback)    

    pub = rospy.Publisher('lcd1602', String, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        send_update(pub)
        rate.sleep()

if __name__ == '__main__':
    main()
