#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import csv
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

writer = None
last = None
first = None

def joint_callback(data):
    global last
    global first
    if last == None:
      first = last = data.header.stamp.to_sec()
      return

    n = data.header.stamp.to_sec()
    d = n - last
    last = n
    writer.writerow([(n - first)*1000, d*1000])

def main():
    global writer
    global first

    try:
        rospy.init_node("test_log", anonymous=True, disable_signals=True)

        with open('log.csv', 'wb') as cf:
          writer = csv.writer(cf, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
          writer.writerow(["t", "dt"])
          rospy.Subscriber("joint_states", JointState, joint_callback)
          time.sleep(60)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
