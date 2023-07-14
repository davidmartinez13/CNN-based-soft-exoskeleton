#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, UInt16, Float32, Float32MultiArray

def deg2rad(degree):
    return degree * 3.14159265359/180

def rad2deg(radian):
    return radian * (180 / 3.14159265359)   

def talker():
    pub = rospy.Publisher('servo', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    q_array = Float32MultiArray()
    delta_t = 1
    deg = float(input('enter staring angle'))
    q1 = deg
    q2 = deg
    while not rospy.is_shutdown():
        try:
            q1 = delta_t + q1
            q2 = delta_t + q2
            q_array.data.clear()
            q_array.data.append(q1)
            q_array.data.append(q2)
            rospy.loginfo(q_array.data)
            pub.publish(q_array)
            rate.sleep()
            if q1>=180 or q2>=180:
                break
        except:
            print('Failed to send angle')
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass