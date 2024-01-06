#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped,PoseStamped 
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


from usv_simulator.msg import UsvCommand 

import rclpy
from rclpy.node import Node

import numpy as np
import math
import os

import webbrowser
import matplotlib.pyplot as plt
from python_vehicle_simulator.vehicles import *
from python_vehicle_simulator.lib import *




def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class BoatSimulator(Node):

    def __init__(self):
        super().__init__('boat_simulator')
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1000)
        self.subscription_pose = self.create_subscription(
            UsvCommand,
            'usv_command',
            self.commande_callback,
            1000)
        self.marker_pub = self.create_publisher(Marker, "helios",100 )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster_ned = TransformBroadcaster(self)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pose = PoseStamped()
        self.marker=Marker()
        self.pose.header.frame_id="map"
        self.marker.header.frame_id="boat"
        self.marker.type = 10
        self.marker.id=0
        self.marker.action = 0
        self.marker.scale.x = 0.004
        self.marker.scale.y = 0.004
        self.marker.scale.z = 0.004
        self.marker.color.r=0.8
        self.marker.color.g=0.8
        self.marker.color.b=0.8
        self.marker.color.a = 0.8

        q_mark=quaternion_from_euler(0,0,np.pi/2)
        self.marker.pose.orientation.z = q_mark[2]
        self.marker.pose.orientation.w =q_mark[3]
        self.marker.mesh_resource="package://usv_simulator/model.obj"



        self.comm=0.
        self.theta,self.x,self.y=0.,0.,0.
        self.moteur=True

        self.vehicle = otter('manualInput',100.0,0.0,-30.0,200.0)
        self.sampleTime = 0.02 

        self.DOF = 6                     # degrees of freedom
        self.t = 0                       # initial simulation time
        # Initial state vectors
        self.eta = np.array([0, 0, 0, 0, 0, 0], float)    # position/attitude, user editable
        self.nu = self.vehicle.nu                              # velocity, defined by vehicle class
        self.u_actual = self.vehicle.u_actual                  # actual inputs, defined by vehicle class
        self.i=0

        self.n1,self.n2=-50,-50


    def one_step(self,eta,nu,u_control,u_actual):
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = self.vehicle.dynamics(eta,nu,u_actual,u_control,self.sampleTime)
        eta = attitudeEuler(eta,nu,self.sampleTime)
        return eta,nu,u_actual,signals
    
        

    def timer_callback(self):
        self.t = self.i * self.sampleTime      # simulation time

        if (self.vehicle.controlMode == 'manualInput'):
            u_control = self.vehicle.manualInput(self.n1,self.n2)    #right,left
        else:
            u_control = np.array([0., 0.], float)   
        
        # Store simulation data in simData
        self.eta,self.nu,self.u_actual,signal=self.one_step(self.eta,self.nu,u_control,self.u_actual)
        self.i+=1
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.x=signal[0]
        self.pose.pose.position.y=signal[1]
        self.pose.pose.position.z=signal[2]
        phi = ssa(signal[3])
        theta = ssa(signal[4])
        psi = ssa(signal[5])

        q=quaternion_from_euler(phi,theta,psi)
        self.pose.pose.orientation.x=q[0]
        self.pose.pose.orientation.y=q[1]
        self.pose.pose.orientation.z=q[2]
        self.pose.pose.orientation.w=q[3]

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'boat'
        t.transform.translation.x = self.pose.pose.position.x
        t.transform.translation.y =self.pose.pose.position.y
        t.transform.translation.z = self.pose.pose.position.z
        t.transform.rotation.x = self.pose.pose.orientation.x
        t.transform.rotation.y = self.pose.pose.orientation.y
        t.transform.rotation.z = self.pose.pose.orientation.z
        t.transform.rotation.w = self.pose.pose.orientation.w

        

        self.tf_broadcaster.sendTransform(t)
        self.pose_publisher.publish(self.pose)
        self.marker_pub.publish(self.marker)




    
    def commande_callback(self,msg):
        if msg.right>100:
            self.n1=100
        elif msg.right<-100:
            self.n1=-100
        else:
            self.n1=msg.right
        if msg.left>100:
            self.n2=100
        elif msg.left<-100:
            self.n2=-100
        else:
            self.n2=msg.left


def main(args=None):
    rclpy.init(args=args)

    boat_simulator = BoatSimulator()

    rclpy.spin(boat_simulator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
