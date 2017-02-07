#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import Float64

UPDATE_RATE = 100.0 #Hz
AMPLITUDE = 50.0 #mm
FREQ = 2.0 #Hz
HEIGHT = 150.0 #mm

class JoyBridge():
    def __init__(self):
        self.L = 93.042124
        self.l = 29.798240
        self.l1 = 90.0
        self.l2 = 140.0
        self.z_offset = -19.293545

        self.x = 0.0
        self.y = 0.0
        self.z = HEIGHT + self.z_offset

        self.q1 = 1.0
        self.q2 = 1.0
        self.q3 = 1.0

        rospy.init_node('joy_bridge', anonymous=True)

        self.j1 = rospy.Publisher("/joint1_controller/command",
            Float64, queue_size=1)
        self.j2 = rospy.Publisher("/joint2_controller/command",
            Float64, queue_size=1)
        self.j3 = rospy.Publisher("/joint3_controller/command",
            Float64, queue_size=1)

    def _inv_kinematics(self):
        a1 = self.x**2 + self.y**2 + self.z**2  \
          + 2*self.y*self.l/sqrt(3) - 2*self.y*self.L/sqrt(3) + self.l**2/3  \
          - 2*self.l*self.L/3 + self.L**2/3 + self.l1**2 - self.l2**2
        b1 = 2*self.y*self.l1 + 2*self.l*self.l1/sqrt(3) - 2*self.l1*self.L/sqrt(3)
        c1 = 2*self.z*self.l1

        self.q1 = atan2(c1,b1) - acos(a1/sqrt(b1**2+c1**2))

        a2 = self.x**2 + self.y**2 + self.z**2 + self.x*self.l - self.x*self.L \
        - self.y*self.l/sqrt(3) + self.y*self.L/sqrt(3) + self.l**2/3 - \
          2*self.l*self.L/3 + self.L**2/3 + self.l1**2 - self.l2**2
        b2 = sqrt(3)*self.x*self.l1 + 2*self.l*self.l1/sqrt(3) - \
        2*self.L*self.l1/sqrt(3) - self.y*self.l1
        c2 = 2*self.z*self.l1

        self.q2 = atan2(c2,b2) - acos(a2/sqrt(b2**2+c2**2))

        a3 = self.x**2 + self.y**2 +self.z**2 - self.x*self.l + self.x*self.L - \
        self.y*self.l/sqrt(3) + self.y*self.L/sqrt(3) + self.l**2/3 - \
        2*self.l*self.L/3 + self.L**2/3 + self.l1**2 - self.l2**2
        b3 = -sqrt(3)*self.x*self.l1 - self.y*self.l1 + \
        2*self.l*self.l1/sqrt(3) - 2*self.L*self.l1/sqrt(3)
        c3 = 2*self.z*self.l1

        self.q3 = atan2(c3,b3) - acos(a3/sqrt(b3**2+c3**2))

    def _send_setpoint(self):
        self.j1.publish(-self.q1-8*pi/180)
        self.j2.publish(-self.q2)
        self.j3.publish(-self.q3-18*pi/180)

    def _circle_traj(self, t):
        self.x = AMPLITUDE*sin(FREQ*(2*pi)*t)
        self.y = AMPLITUDE*cos(FREQ*(2*pi)*t)
        self._inv_kinematics()

    def run(self):
        timer = rospy.Rate(UPDATE_RATE)
        time = 0.0
        while not rospy.is_shutdown():
            self._circle_traj(time)
            self._send_setpoint()
            timer.sleep()
            time = time + 1.0/UPDATE_RATE

if __name__ == '__main__':
    bridge = JoyBridge()
    bridge.run()
