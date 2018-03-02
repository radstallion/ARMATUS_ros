#!/usr/bin/env python
import time
import ikpy
import numpy as np
import os, rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import rospy


class virtual_impedance:
    def __init__(self,mass,spring,damper):
        self.mass = mass
        self.spring = spring
        self.damper = damper
        self.dis = np.array([0.0,0.0,0.0]) #displacement
        self.velo = np.array([0.0,0.0,0.0]) #velocity
        self.acc = np.array([0.0,0.0,0.0]) #accerelation
        
    def run(self,force,dt):
        _force = np.subtract(force, np.add(np.multiply(self.spring,self.dis), np.multiply(self.damper,self.velo)) )#will change to force input
        self.acc = _force/self.mass
        self.velo += self.acc*dt
        self.dis += self.velo*dt 
        return self.dis

class joint_publisher:
    def __init__(self,joint_name=['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],name='joint_state_publisher'):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node(name)
        self.message = JointState()
        self.message.header = Header()
        self.message.name = joint_name

    def send(self,joints):
        self.message.header.stamp = rospy.Time.now()
        self.message.position = joints[1:]
        self.pub.publish(self.message)
        

class robotIK:
    def __init__(self,urdf_path,home):
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        self.home = home

        # target_vector = [ 0, 0.25, 0.4]
        target_frame = np.eye(4)
        target_frame[:3, 3] = home
        self.current_joint = self.chain.inverse_kinematics(target_frame)

    def forwardIK(self,joint):

        return self.chain.forward_kinematics(joint)

    def inverseIK(self,pose,current_joint=None):

        target_frame = np.eye(4)
        target_frame[:3, 3] = pose

        if current_joint is None:
            target_joint = self.chain.inverse_kinematics(target_frame)
        else:
            target_joint = self.chain.inverse_kinematics(target_frame,joint)

        return target_joint




#initialize stuffs

VID = virtual_impedance(3.0,15.0,0.5)
pub = joint_publisher()

# URDFIK
rp = rospkg.RosPack()
urdf_path = os.path.join(rp.get_path("control"), "urdf", "kuka_bot.urdf")
robot = robotIK(urdf_path,[ 0, 0.25, 0.4])
joint = robot.current_joint


#simulation configuration
force = np.array([3.0,3.0,3.0])
dt = 0.132 # 132ms
iteration = 1000

# the loop

for i in range(iteration):
    t = time.time()

    #virtual impedance
    dis = VID.run(force,dt)

    joint = robot.inverseIK(robot.home + dis,joint)

    #######execute to robot here#######
    pub.send(joint)
    ###################################

    dt = time.time() - t

    print("total elapse time : ",dt )
