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
    def __init__(class,joint_name=['q1', 'q2', 'q3', 'q4', 'q5', 'q6'],name='joint_state_publisher'):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node(name)
        self.message = JointState()
        self.message.header = Header()
        self.message.name = joint_name

    def send(self,joints):
        self.message.header.stamp = rospy.Time.now()
        self.message.position = joints[1:]
        self.pub.publish(self.message)
        



# from ikpy import plot_utils
rp = rospkg.RosPack()
script_path = os.path.join(rp.get_path("control"), "urdf", "kuka_bot.urdf")

my_chain = ikpy.chain.Chain.from_urdf_file(script_path)

#initialize stuffs

VID = virtual_impedance(3.0,15.0,0.5)
pub = joint_publisher()



#starting pose
target_vector = [ 0, 0.25, 0.4]
target_frame = np.eye(4)
target_frame[:3, 3] = target_vector

joint = my_chain.inverse_kinematics(target_frame)
real_frame = my_chain.forward_kinematics(joint)

starting_pos = real_frame[:3, 3]

#simulation configuration
force = np.array([0.0,3.0,3.0])
dt = 0.132 # 132ms
iteration = 10000
displace = np.zeros([iteration,3])
joints = np.zeros([iteration,7])
_positions = np.zeros([iteration,3])

# the loop

for i in range(iteration):
    t = time.time()

    #virtual impedance
    dis = VID.run(force,dt)
    
    target_pos = target_vector + dis
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_pos
    joint = my_chain.inverse_kinematics(target_frame,joint)
    joints[i] = joint


    #######execute to robot here#######
    pub.send(ros_joint_states)
    ###################################

    real_frame = my_chain.forward_kinematics(joints[i])
    current_pos = real_frame[:3, 3]
    _positions[i] = current_pos
    
    dt = time.time() - t

    print("total elapse time : ",dt )



# fig = plt.figure()
# ax = fig.add_subplot(311)
# ax.plot(range(iteration), displace)
# ax1 = fig.add_subplot(312)
# ax1.plot(range(iteration), joints)
# ax2 = fig.add_subplot(313)
# ax2.plot(range(iteration), _positions)

# plt.show()
