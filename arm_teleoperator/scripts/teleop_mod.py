#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, TwistStamped
import std_msgs.msg

import sys, select, termios, tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
    x-axis:   x (+1)   c (-1)     
    y-axis:   y (+1)   u (-1)
    z-axis:   z (+1)   a (-1)  
    
Angular motion:
    pitch (theta): p (+1) o (-1)
    yaw (psi)    : i (+1) k (-1)
    roll (phi)   : r (+1) t (-1) 

anything else : stop

CTRL-C to quit
"""


moveBindings = {
    
        'x':(1, 0, 0, 0, 0, 0),
        'c':(-1, 0, 0, 0, 0, 0),
        'y':(0, 1, 0, 0, 0, 0),
        'u':(0, -1, 0, 0, 0, 0),
        'z':(0, 0, 1, 0, 0, 0),
        'a':(0, 0, -1, 0, 0, 0),
        'p':(0, 0, 0, 1, 0, 0),
        'o':(0, 0, 0, -1, 0, 0),
        'i':(0, 0, 0, 0, 1, 0),
        'k':(0, 0, 0, 0, -1, 0),
        'r':(0, 0, 0, 0, 0, 1),
        't':(0, 0, 0, 0, 0, -1),
        

    }



class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('eef/pose', TwistStamped, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.phi = 0.0
       
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, theta, psi, phi):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.psi = psi
        self.phi = phi
        
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        cmd = TwistStamped()
        
        # Insert the subcriber functionality to accuqire the current pose of the end-effector
        # Now I am initializing randomanly
        
        current_x = 3
        current_y = 3
        current_z = 3
        current_theta = 13
        current_psi = 3
        current_phi = 3
        
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Header definition
            cmd.header.stamp = rospy.Time.now()
            
            # Copy state into twist message.
            cmd.twist.linear.x = self.x + current_x
            current_x = cmd.twist.linear.x
            
            cmd.twist.linear.y = self.y + current_y
            current_y = cmd.twist.linear.y
            
            cmd.twist.linear.z = self.z + current_z
            current_z = cmd.twist.linear.z
            
            cmd.twist.angular.x = self.theta + current_theta
            current_theta = cmd.twist.angular.x 
            
            cmd.twist.angular.y = self.psi + current_psi
            current_psi = cmd.twist.angular.y
            
            cmd.twist.angular.z = self.phi + current_phi
            current_phi = cmd.twist.angular.z
            
            self.condition.release()

            # Publish.
            self.publisher.publish(cmd)

        # Publish stop message when thread exits.
        cmd.twist.linear.x = 0
        cmd.twist.linear.y = 0
        cmd.twist.linear.z = 0
        cmd.twist.angular.x = 0
        cmd.twist.angular.y = 0
        cmd.twist.angular.z = 0
        self.publisher.publish(cmd)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key




if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    # speed = rospy.get_param("~speed", 0.5)
    # turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
 
    # current_x = 3   # to have the track of current x pos which we will obtain from fwd_kin
    x = 0
    y = 0
    z = 0
    theta = 0
    psi = 0
    phi = 0
    
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, theta, psi, phi)

        print(msg)
        
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                theta = moveBindings[key][3]
                psi = moveBindings[key][4]
                phi = moveBindings[key][5]
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and theta == 0:
                    continue
                x = 0
                y = 0
                z = 0
                theta = 0
                psi = 0
                phi = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, theta, psi, phi)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)