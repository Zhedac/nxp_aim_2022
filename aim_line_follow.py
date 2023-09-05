import rclpy
from rclpy.node import Node
from math import exp,sqrt
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import Parameter
# from rcl_interfaces.msg import ParameterType
# from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from nxp_cup_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
import numpy as np

class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')

        self.state = 0		#check if car is on the overbridge

        self.state1_current_time = 0		#last time on the overbridge

        self.initial = 0 # it will be used in calculating dt in our PID function
        self.pid_error = [0,0] # it will be used in storing previous error in our PID function
        self.pid_sum = [0,0] # it will be used in storing error sum in our PID function

        self.odom = False # when set to true, it will do a speed control, since the actual speed may become more(uphill) or less(downhill) than the given speed
        
        self.start_delay = 5.0
        
        self.camera_vector_topic = "/cupcar0/PixyVector"
        
        self.linear_velocity = 1.6 

        self.a = 1.6 # our exponential decay term for our speed control function during steer
        
        self.angular_velocity = 0.6 
        
        self.single_line_steer_scale = 1.0 	#steer scale when one end-track detected 

        self.double_line_steer_scale = 1.0 	#steer scale when two end-tracks detected 
        
        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)
        self.middlesub = self.create_subscription(String,"/middle",self.middle,10) # get the message of average white pixel densities on left and right of car
        self.distancesub = self.create_subscription(Float32,"/cupcar0/distance",self.distance,10) # to know the distance the car has travelled
        self.odomsub = self.create_subscription(Odometry,"/cupcar0/odom",self.speed_check,10) # to access vehicle speed readings

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)
		#publisher of velocity
		
        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        # Timer setup
        # timer_period = 0.5 #seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def get_num_vectors(self, msg):
        num_vectors = 0
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        return num_vectors

    # def timer_callback(self):
    #     #TODO

    def listener_callback(self, msg):
        #TODO
        current_time = datetime.now().timestamp()
        frame_width = 79
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        num_vectors = self.get_num_vectors(msg)

        if(num_vectors == 0):
            if self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = False
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * (4.0-(current_time-self.start_time))/4.0
            if (self.start_time+4.0) <= current_time:
                speed = 0.6
            steer = 0
        
        if(num_vectors == 1):
            
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            if(msg.m0_x1 > msg.m0_x0):
                x = (msg.m0_x1 - msg.m0_x0) / frame_width
                y = (msg.m0_y1 - msg.m0_y0) / frame_height
            else:
                x = (msg.m0_x0 - msg.m0_x1) / frame_width
                y = (msg.m0_y0 - msg.m0_y1) / frame_height
            if(msg.m0_x0 != msg.m0_x1 and y != 0):
                steer = (self.angular_velocity) * (x / y) * (self.single_line_steer_scale)
                if (self.start_time+4.0) > current_time:
                    speed = (self.linear_velocity) * ((current_time-self.start_time)/4.0)
                if (self.start_time+4.0) <= current_time:
                    speed = self.linear_velocity
            else:
                steer = 0
                if (self.start_time+4.0) > current_time:
                    speed = (self.linear_velocity) * ((current_time-self.start_time)/4.0)*0.9
                if (self.start_time+4.0) <= current_time:
                    speed = (self.linear_velocity)
    
            

        if(num_vectors == 2):
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            m_x1 = (msg.m0_x1 + msg.m1_x1) / 2
            steer = -(self.angular_velocity)*(m_x1 - window_center) * (self.double_line_steer_scale) / frame_width
        
            if (self.start_time+4.0) > current_time:
                speed = (self.linear_velocity) * ((current_time-self.start_time)/4.0)
            if (self.start_time+4.0) <= current_time:
                speed = (self.linear_velocity)
            if (msg.m0_x0<window_center and msg.m1_x0<window_center):	
                steer=-0.3		 #if both num_vectors are on the left side from the center
            if (msg.m0_x0>window_center and msg.m1_x0>window_center):
                steer=0.3   		#if both num_vectors are on the right side from the center

            if (self.state ==0):		#checking if car is on the bridge
                if (msg.m0_y0>msg.m0_y1 and msg.m1_y0>msg.m1_y1):
                    if (msg.m0_x1>26 and msg.m1_x1<52 and msg.m0_x0<msg.m0_x1 and msg.m1_x1<msg.m1_x0 and (msg.m0_x0-window_center)*(msg.m1_x0-window_center)<0 
                    and datetime.now().timestamp()-self.state1_current_time>2):
                        self.state = 1	#car is on the bridge
                        self.state1_current_time = datetime.now().timestamp()	#updating state1_current_time to current time

            if (self.state ==1):
                speed = (self.linear_velocity)
                steer = (msg.m1_y0 - msg.m0_y0)/25		#deciding steer from the difference in y values of the tails of vectors
                if ((msg.m0_x0-msg.m0_x1)!=0 and (msg.m1_x0-msg.m1_x1)!=0):
                    m1 = float((msg.m0_y0 - msg.m0_y1)/(msg.m0_x0-msg.m0_x1))	#slope of the left vector
                    m2 = float((msg.m1_y0 - msg.m1_y1)/(msg.m1_x0-msg.m1_x1))	#slope of the right vector
                    steer = float((m1+m2)/1.5)					#steer the car at the average of m1 and m2
                if (datetime.now().timestamp()-self.state1_current_time>5):
                    self.state = 0					#change state back to 'not on overbridge' after some time

            if (self.state ==1):
                if (msg.m0_x1<20 and msg.m1_x1>60 and msg.m0_x0<msg.m0_x1 and msg.m1_x1<msg.m1_x0):
                    self.state = 0					#not on overbridge condition
        if (self.state ==1):
            speed *= 1.1					#increasing speed on th upclimb of bridge
            if ((datetime.now().timestamp()-self.state1_current_time>0.5)):
                self.state = 0				#change state back to 'not on overbridge' after some time
                self.state1_current_time = datetime.now().timestamp()    
        
        # speed = speed*e^(-a*|steer|)                              #reducing speed at sharp steers
        self.speed_vector.x = min(float(speed*exp(-self.a*np.abs(steer))),self.linear_velocity) # our speed control function using steer
        self.steer_vector.z = float(steer)

        
    def middle(self,data):
        # this function will add an additional steer to the car based on the white pixel densities on left and right. a high density compared to other means
        # that the road is on that way and the car should steer towards it
        d = data.data.split("%")
        l = float(d[0]) # density at left
        r = float(d[1]) # density at right
        pid = self.PID((l,r),1.0,0.3,0) # using pid function for stable performance
        self.steer_vector.z = self.steer_vector.z + (pid[0] - pid[1])/2.0 # if density at left is higher, car should move anticlockwise and therefore steer should
        # be +ve and hence it is added. for right steer should be negative and hence it is subtracted

    def distance(self,data):
        distance = data.data
        if distance < 23: # if we are going through smooth track, we can speed up
            self.linear_velocity = 2.0
            self.a = 2.5
        elif(distance > 43 and distance < 47) or (distance > 61 and distance < 65): # during uphill, slow down a bit
            self.linear_velocity = 1.0
            self.a = 1.0
            self.odom = True # we need a speed control since speed will decrease than its supplied value
        elif(distance >= 47 and distance < 50) or (distance >= 65 and distance < 67): # during downhill, we need to slow more
            self.linear_velocity = 0.6
            self.a = 0.6
            self.odom = True # we need a speed control since speed will increase than its supplied value
        elif(distance > 101 and distance < 108): # during bumpy road, slow down a bit
            self.odom = True
            self.linear_velocity = 0.8
            self.a = 0.8
        else: # in other cases, just move
            self.linear_velocity = 1.6
            self.a = 1.6
            self.odom = False # we don't need speed control now

    def speed_check(self,data):
        kp = 0.1 # proportional coefficient
        if (self.odom):
            vel_x = data.twist.twist.linear.x
            vel_y = data.twist.twist.linear.y
            error = self.speed_vector.x - sqrt((vel_x**2) + (vel_y**2)) # finding the net increment/decrement in our speed as an error
            self.speed_vector.x = self.speed_vector.x + kp*error # feeding it in speed to control it
        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector
        self.cmd_vel_publisher.publish(self.cmd_vel) # publishing speed

    def PID(self, X,kp,kd,ki):
        dt = datetime.now().timestamp() - self.initial # smal time interval
        output = [0,0] # since 2 pid will be calculated on the go, one for left density and one for right density
        for i,x in enumerate(X):
            self.pid_sum[i] = self.pid_sum[i] + x*dt # integral sum
            output[i] = kp*x + kd*(x - self.pid_error[i])/dt + ki*self.pid_sum[i] # output for each density
            self.pid_error[i] = x # storing error for next time
        self.initial = datetime.now().timestamp() # storing time for next
        return output

        
def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
