#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperEulerPose
import message_filters
import tf.transformations as tf_trans


class EndPoseDeltaNode:
    """
    Subscribe:
      - end_pose            (PoseStamped)
      - end_pose_euler      (PiperEulerPose)

    Publish:
      - end_pose_delta        (PoseStamped)
      - end_pose_euler_delta (PiperEulerPose)
    """

    def __init__(self):
        rospy.init_node("end_pose_delta_node")

        self.topic_prefix = rospy.get_param("~topic_prefix", "")
        if self.topic_prefix and not self.topic_prefix.endswith("/"):
            self.topic_prefix += "/"

        # ---------------- Publisher ----------------
        self.pose_delta_pub = rospy.Publisher(
            self.topic_prefix + "end_pose_delta",
            PoseStamped,
            queue_size=10,
        )
        self.euler_delta_pub = rospy.Publisher(
            self.topic_prefix + "end_pose_euler_delta",
            PiperEulerPose,
            queue_size=10,
        )

        # ---------------- Cache ----------------
        self.prev_pose = None
        self.prev_euler = None
        self.lock = threading.Lock()

        # ---------------- Subscriber (time sync) ----------------
        pose_sub = message_filters.Subscriber(
            self.topic_prefix + "end_pose", PoseStamped
        )
        euler_sub = message_filters.Subscriber(
            self.topic_prefix + "end_pose_euler", PiperEulerPose
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, euler_sub],
            queue_size=20,
            slop=0.01,          # 10ms 时间容忍
            allow_headerless=False,
        )
        self.ts.registerCallback(self.synced_callback)

        rospy.loginfo("[end_pose_delta_node] started.")
        rospy.spin()

    # ==========================================================
    # Helper functions
    # ==========================================================
    def quaternion_to_euler(self, q):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        # 注意：这里假设旋转顺序是XYZ (roll, pitch, yaw)
        return tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w], axes='sxyz')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        q = tf_trans.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
        return q
    
    def angle_difference(self, a, b):
        """计算两个角度之间的最小差值（处理角度包装）"""
        diff = a - b
        # 将差值映射到 [-pi, pi] 区间
        return (diff + np.pi) % (2 * np.pi) - np.pi
    
    def compute_relative_pose(self, current_pose, prev_pose):
        """计算从prev_pose到current_pose的相对变换"""
        # 位置差（基座标系下）
        delta_pos = [
            current_pose.position.x - prev_pose.position.x,
            current_pose.position.y - prev_pose.position.y,
            current_pose.position.z - prev_pose.position.z
        ]
        
        # 姿态差（计算相对四元数）
        q1 = [
            prev_pose.orientation.x,
            prev_pose.orientation.y,
            prev_pose.orientation.z,
            prev_pose.orientation.w
        ]
        q2 = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        
        # 计算相对旋转：q_rel = q2 * q1_conjugate
        q1_conj = tf_trans.quaternion_conjugate(q1)
        q_rel = tf_trans.quaternion_multiply(q2, q1_conj)
        
        # 确保四元数归一化
        q_rel = tf_trans.unit_vector(q_rel)
        
        return delta_pos, q_rel

    # ==========================================================
    # Callback
    # ==========================================================
    def synced_callback(self, pose: PoseStamped, euler: PiperEulerPose):
        with self.lock:
            # 第一帧只缓存
            if self.prev_pose is None or self.prev_euler is None:
                self.prev_pose = pose
                self.prev_euler = euler
                return

            # ---------------- 计算Pose delta ----------------
            pose_delta = PoseStamped()
            pose_delta.header.stamp = pose.header.stamp
            pose_delta.header.frame_id = pose.header.frame_id

            # 位置直接相减（基座标系下）
            pose_delta.pose.position.x = (
                pose.pose.position.x - self.prev_pose.pose.position.x
            )
            pose_delta.pose.position.y = (
                pose.pose.position.y - self.prev_pose.pose.position.y
            )
            pose_delta.pose.position.z = (
                pose.pose.position.z - self.prev_pose.pose.position.z
            )

            # 四元数相对旋转（正确方法）
            delta_pos, q_rel = self.compute_relative_pose(
                pose.pose, self.prev_pose.pose
            )
            
            pose_delta.pose.orientation.x = q_rel[0]
            pose_delta.pose.orientation.y = q_rel[1]
            pose_delta.pose.orientation.z = q_rel[2]
            pose_delta.pose.orientation.w = q_rel[3]

            self.pose_delta_pub.publish(pose_delta)

            # ---------------- 计算Euler delta ----------------
            euler_delta = PiperEulerPose()
            euler_delta.header.stamp = pose.header.stamp

            # 位置差
            euler_delta.x = pose.pose.position.x - self.prev_pose.pose.position.x
            euler_delta.y = pose.pose.position.y - self.prev_pose.pose.position.y
            euler_delta.z = pose.pose.position.z - self.prev_pose.pose.position.z

            # 方法1：从相对四元数计算欧拉角差值（推荐）
            # 将相对四元数转换为欧拉角
            roll, pitch, yaw = tf_trans.euler_from_quaternion(q_rel, axes='sxyz')
            euler_delta.roll = roll
            euler_delta.pitch = pitch
            euler_delta.yaw = yaw

            # 方法2：处理角度包装的直接相减（如果需要保持原欧拉角输入）
            # 注意：这种方法只在角度变化很小且没有奇异点时准确
            # euler_delta.roll = self.angle_difference(euler.roll, self.prev_euler.roll)
            # euler_delta.pitch = self.angle_difference(euler.pitch, self.prev_euler.pitch)
            # euler_delta.yaw = self.angle_difference(euler.yaw, self.prev_euler.yaw)

            self.euler_delta_pub.publish(euler_delta)

            # ---------------- Update cache ----------------
            self.prev_pose = pose
            self.prev_euler = euler


if __name__ == "__main__":
    EndPoseDeltaNode()