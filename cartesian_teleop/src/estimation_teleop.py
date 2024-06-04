#!/usr/bin/python

import sys
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
import sensor_msgs.msg as sense_msg
import control_msgs.msg as ctrl_msg
import tf2_msgs.msg as tf2_msg
import tf2_ros
from std_srvs.srv import Empty
import time
import subprocess
from absl import app, flags, logging
from scipy.spatial.transform import Rotation as R
from pydrake.all import (
    RigidTransform, Quaternion, RollPitchYaw
)

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_ip", None, "IP address of the robot.", required=True)
flags.DEFINE_string("load_gripper", 'false',
                    "Whether or not to load the gripper.")


def _write_pose_msg(X_AB, p, q):
    # p - position message
    # q - quaternion message
    X_AB = RigidTransform(X_AB)
    p.x, p.y, p.z = X_AB.translation()
    q.w, q.x, q.y, q.z = X_AB.rotation().ToQuaternion().wxyz()


def to_ros_transform(X_AB):
    """Converts Drake transform to ROS transform."""
    msg = geom_msg.Transform()
    _write_pose_msg(X_AB, p=msg.translation, q=msg.rotation)
    return msg


def to_ros_pose(X_AB):
    """Converts Drake transform to ROS pose."""
    msg = geom_msg.Pose()
    _write_pose_msg(X_AB, p=msg.position, q=msg.orientation)
    return msg


class SpacenavTeleOperator(object):
    """
    This class is responsible for relaying spacenav twists as equilibrium poses
    to the cartesian controller

    """

    def __init__(self, name, debug=True):
        # initialisation message
        self._name = name
        rospy.loginfo("%s: Spacenav teleop node initialized", self._name)
        self._debg_cnt = 0
        self._trans_scale = 0.05
        self._ori_scale = 0.2
        self._debug = debug

        # Declare subscribers
        self._joy_sub = rospy.Subscriber(
            "/spacenav/joy",
            sense_msg.Joy,
            self._joyCallback
        )
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        self._tfBr = tf2_ros.TransformBroadcaster()
        # self._equilibrium_pose_sub = rospy.Subscriber(
        #     "/cartesian_impedance_controller/equilibrium_pose",
        #     geom_msg.PoseStamped,
        #     self._poseCallback
        # )
        self._equilibrium_pose_pub = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose",
            geom_msg.PoseStamped,
            queue_size=1
        )
        self._gripper_cmd_pub = rospy.Publisher(
            "/gripper_controller/gripper_cmd/goal",
            ctrl_msg.GripperCommandActionGoal,
            queue_size=1
        )

    def _joyCallback(self, msg):
        # TODO: remove hard coded tf frame ids
        button1 = msg.buttons[0]
        button2 = msg.buttons[1]
        if button1:
            # send a static transform
            # start the estimation service
            rospy.loginfo("Button press event: button 1")
            action = ctrl_msg.GripperCommandActionGoal()
            action.header.stamp = rospy.Time.now()
            goal = ctrl_msg.GripperCommandGoal()
            cmd = ctrl_msg.GripperCommand()
            cmd.position = 0.6
            cmd.max_effort = 50
            goal.command = cmd
            action.goal = goal
            self._gripper_cmd_pub.publish(action)
        if button2:
            # stop the estimation service
            # report all estimated variables
            rospy.loginfo("Button press event: button 2")
            action = ctrl_msg.GripperCommandActionGoal()
            action.header.stamp = rospy.Time.now()
            goal = ctrl_msg.GripperCommandGoal()
            cmd = ctrl_msg.GripperCommand()
            cmd.position = 0
            cmd.max_effort = 50
            goal.command = cmd
            action.goal = goal
            self._gripper_cmd_pub.publish(action)

        try:
            tf_X_EE = self._tfBuffer.lookup_transform(
                'panda_link0',
                'panda_EE',
                rospy.Time())
            translation = tf_X_EE.transform.translation
            ori = tf_X_EE.transform.rotation

            dr_quat = Quaternion(wxyz=np.array([ori.w, ori.x, ori.y, ori.z]))

            # gets current pose of EE
            X_Eeff = RigidTransform(dr_quat, np.array(
                [translation.x,
                 translation.y,
                 translation.z]))

            # fixes a fixed frame to the end-effector
            X_Fixeff = RigidTransform(p=np.array(
                [translation.x,
                 translation.y,
                 translation.z]))

            # transformation between fixed end effector frame and End effector
            # frame
            X_FixeffEeff = X_Fixeff.inverse() @ X_Eeff
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        # extracts joy stick information
        x = msg.axes[0] * self._trans_scale
        y = msg.axes[1] * self._trans_scale
        z = msg.axes[2] * self._trans_scale
        roll = msg.axes[3] * self._ori_scale
        pitch = msg.axes[4] * self._ori_scale
        yaw = msg.axes[5] * self._ori_scale

        # constructs transformation of new fixed end effector frame from joy
        # stick information w.r.t fixed end effector frame
        X_Fixeff_Fixeffnew = RigidTransform(
            rpy=RollPitchYaw([roll, pitch, yaw]),
            p=[x, y, z])

        # Spatial algebra to extract new Fixed end effector w.r.t world
        X_Fixeffnew = X_Fixeff @ X_Fixeff_Fixeffnew

        # spatial algebra to get new command end effector frame w.r.t world
        # QUE: Why is this working?
        X_Eeffnew = X_Fixeffnew @ X_FixeffEeff

        # sets tf with command pose
        t = geom_msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "panda_link0"
        t.child_frame_id = "wp"
        t.transform = to_ros_transform(X_Eeffnew)
        self._tfBr.sendTransform(t)

        if not self._debug:
            # sends calculated pose to robot
            pose = geom_msg.PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "e_pose"
            pose.pose = to_ros_pose(X_Eeffnew)

            self._equilibrium_pose_pub.publish(pose)

    def _poseCallback(self, msg):
        rospy.loginfo("Current equilibrium pose of the robot: \n %s"
                      % (msg.Pose.position))


if __name__ == "__main__":
    rospy.init_node('teleop', anonymous=True)
    teleop_client = SpacenavTeleOperator(rospy.get_name(), False)
    rospy.spin()
    # app.run(main)
