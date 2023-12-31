#!/usr/bin/env python3
import rospy
import time
import multiprocessing
from std_msgs.msg import String, UInt16, Float32, Int16MultiArray, Float32MultiArray

class ExoControl():
    def __init__(self, delta_t = 5):
        self.pub = rospy.Publisher('servo', Int16MultiArray, queue_size=10)
        # self.pub = rospy.Publisher('servo', Float32MultiArray, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        # self.q_array = Float32MultiArray()
        self.q_array = Int16MultiArray()
        self.q1 = 10
        self.q2 = 10
        self.delta_t = delta_t
        self.is_out_of_range = False
        self.command_open = True
        self.hand_opening = 'n'
        mode = str(input('Exo control mode selection: \n 1: command based \n 2: comand smooth \n 3: test mode \n 4: neural net mode \n'))
        if mode == '1':
            self.control_func = self.command_mode

        elif mode == '2':
            self.control_func = self.command_mode_smooth

        elif mode == '3':
            deg = float(input('enter starting angle: '))
            self.q1 = deg
            self.q2 = deg
            self.control_func = self.test_mode

        elif mode == '4':
            # self.ai_mode()
            self.ai_listener()
            self.neural_net_mode_discrete()
            self.control_func = self.test_mode

        else :
            print('Choose a valid Mode')

    def ai_mode(self):
        # overkill, probably not needed
        # Create separate processes for publisher and user input loops
        publisher_process = multiprocessing.Process(target=self.neural_net_mode)
        ai_input_process = multiprocessing.Process(target=self.ai_listener)

        # Start both processes
        ai_input_process.start()
        publisher_process.start()

        # Wait for both processes to finish
        publisher_process.join()
        ai_input_process.join()

    def deg2rad(self, degree):
        return degree * 3.14159265359/180

    def rad2deg(self, radian):
        return radian * (180 / 3.14159265359)

    def ai_callback(self, hand_opening):
        # print('got data')
        self.hand_opening = hand_opening.data
        
    def ai_listener(self):
        # rospy.init_node('opening_listener', anonymous=True)
        print('started ai listener')
        rospy.Subscriber("hand_opening", String, self.ai_callback)

    def neural_net_mode(self):
        print('started ai mode')
        while not rospy.is_shutdown():
            try:
                print(self.hand_opening)
                if self.hand_opening != 'n':
                    self.command_open = not self.command_open
                    print(self.command_open)

                    angle = 170 if self.command_open is True else 10
                    delta_t = self.delta_t if self.command_open is True else -self.delta_t

                    while self.q1 != angle and self.q2 != angle:
                        self.q1 = self.q1 + delta_t 
                        self.q2 = self.q2 + delta_t 
                        self.q_array.data.clear()
                        self.q_array.data.append(self.q1)
                        self.q_array.data.append(self.q2)
                        rospy.loginfo(self.q_array.data)
                        self.pub.publish(self.q_array)
                        self.rate.sleep()
                    time.sleep(2)
                
            except Exception as e:
                print('Failed to send angle: ',e)

    def neural_net_mode_discrete(self):
        print('started ai mode')
        grasp_done = True
        grasp_started = False
        while not rospy.is_shutdown():
            try:
                if grasp_done and not grasp_started :
                    # print(self.hand_opening)
                    if self.hand_opening == 'xl':
                        angle = 170
                        grasp_done = False
                    elif self.hand_opening == 'l':
                        angle = 130
                        grasp_done = False
                    elif self.hand_opening == 'm':
                        angle = 90
                        grasp_done = False
                    elif self.hand_opening == 's':
                        angle = 50
                        grasp_done = False
                    else:
                        self.q_array.data.clear()
                        self.q_array.data.append(0)
                        self.q_array.data.append(0)
                        rospy.loginfo(self.q_array.data)
                        self.pub.publish(self.q_array)
                        self.rate.sleep()
                
                    if not grasp_done and not grasp_started:
                        grasp_started = True
                        self.q1 = angle
                        self.q2 = angle
                        self.q_array.data.clear()
                        self.q_array.data.append(self.q1)
                        self.q_array.data.append(self.q2)
                        rospy.loginfo(self.q_array.data)
                        self.pub.publish(self.q_array)
                        self.rate.sleep()
                        time.sleep(4)


                        grasp_done = True
                        grasp_started = False
                
            except Exception as e:
                print('Failed to send angle: ',e)

    def test_mode(self):
        self.q1 = self.delta_t + self.q1
        self.q2 = self.delta_t + self.q2
        self.q_array.data.clear()
        self.q_array.data.append(self.q1)
        self.q_array.data.append(self.q2)
        rospy.loginfo(self.q_array.data)
        self.pub.publish(self.q_array)
        self.rate.sleep()
        if self.q1>=170 or self.q2>=170:
            self.is_out_of_range = True

    def command_mode(self):
        key = str(input('press enter to continue'))
        if key == '':
            self.command_open = not self.command_open
            print(self.command_open)

            angle = 170 if self.command_open is True else 10
            delta_t = self.delta_t if self.command_open is True else -self.delta_t
            self.q_array.data.clear()
            self.q_array.data.append(angle)
            self.q_array.data.append(angle)
            rospy.loginfo(self.q_array.data)
            self.pub.publish(self.q_array)
            self.rate.sleep()
        time.sleep(1)

    def command_mode_smooth(self):
        key = str(input('press enter to continue'))
        if key == '':
            self.command_open = not self.command_open
            print(self.command_open)

            angle = 170 if self.command_open is True else 10
            delta_t = self.delta_t if self.command_open is True else -self.delta_t

        while self.q1 != angle and self.q2 != angle:
            self.q1 = self.q1 + delta_t 
            self.q2 = self.q2 + delta_t 
            self.q_array.data.clear()
            self.q_array.data.append(self.q1)
            self.q_array.data.append(self.q2)
            rospy.loginfo(self.q_array.data)
            self.pub.publish(self.q_array)
            self.rate.sleep()
        time.sleep(4)


    def control_loop(self):
        while not rospy.is_shutdown():
            try:
                self.control_func()
                if self.is_out_of_range:
                    rospy.signal_shutdown('angle reached')
            except Exception as e:
                print('Failed to send angle: ',e)
    

def deg2rad(degree):
    return degree * 3.14159265359/180

def rad2deg(radian):
    return radian * (180 / 3.14159265359)   

def talker():
    pub = rospy.Publisher('servo', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    q_array = Float32MultiArray()
    delta_t = 5
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
            if q1>=170 or q2>=170:
                break
        except:
            print('Failed to send angle')
if __name__ == '__main__':
    try:
        # talker()
        ec = ExoControl(delta_t = 5)
        ec.control_loop()
    except rospy.ROSInterruptException:
        pass