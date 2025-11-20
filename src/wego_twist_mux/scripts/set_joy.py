#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class TwistFilter:
    def __init__(self):
        rospy.init_node('twist_filter_node')
        self.epsilon_out = rospy.get_param('~epsilon_out', 0.03)
        self.was_active = False

        self.pub = rospy.Publisher('/cmd_vel/joy', Twist, queue_size=10)
        rospy.Subscriber('/cmd_vel/joy_raw', Twist, self.cb)
        rospy.loginfo("twist_filter_node: epsilon_out=%.3f", self.epsilon_out)

    def clamp(self, v):  # 작은 값 0으로
        return 0.0 if abs(v) < self.epsilon_out else v

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x  = self.clamp(msg.linear.x)
        out.linear.y  = self.clamp(msg.linear.y)
        out.linear.z  = self.clamp(msg.linear.z)
        out.angular.x = self.clamp(msg.angular.x)
        out.angular.y = self.clamp(msg.angular.y)
        out.angular.z = self.clamp(msg.angular.z)

        active = any([
            out.linear.x, out.linear.y, out.linear.z,
            out.angular.x, out.angular.y, out.angular.z
        ])

        if active:
            self.pub.publish(out)
            self.was_active = True
        else:
            if self.was_active:
                # 비영 -> 영 전환시 0 한 번 덮어써서 mux에 확실히 정지 전달
                self.pub.publish(Twist())
                self.was_active = False
            # 이후엔 발행 안 함

if __name__ == '__main__':
    TwistFilter()
    rospy.spin()
