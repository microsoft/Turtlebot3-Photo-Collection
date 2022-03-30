#!/usr/bin/env python3

from azure.cognitiveservices.vision.customvision.training import CustomVisionTrainingClient
from azure.cognitiveservices.vision.customvision.training.models import *
from msrest.authentication import ApiKeyCredentials
from turtlebot3_bonsai.turtlebot3_policy_connection import PolicyConnection

import cv2
import uuid
import time
import math
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Pose, Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation

NODE_NAME="Photo_Collector"

IDLE = 0
MAPPING = 1
PHOTOGRAPHING = 2

# Custom Vision recommends at least 30 images per tag
MAX_UPLOAD_PER_TAG = 30

class PhotoCollector(PolicyConnection):
    def __init__(self):
        super().__init__(policy_url="http://localhost:5000", concept_name="PickOne", node_name="main", sim=False)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('endpoint', None),
                ('training_key', None),
                ('prediction_key', None),
                ('predicition_resource_id', None),
                ('project_id', None),
                ('custom_vision_project_name', None),
                ('state_machine_val', None)
            ]
        )

        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.state_machine = IDLE

        self.object_id = 0
        self.image_num = 0
        self.image_list = []
        self.target_pose = None
        self.distance_to_goal = None
        self.clicked_pose_sub = self.create_subscription(PoseStamped, 
                                                          "/move_base_simple/goal", 
                                                          self._pose_callback, 
                                                          QoSProfile(depth=10))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        credentials = ApiKeyCredentials(in_headers={"Training-key": self.get_parameter('training_key').value})
        self.trainer = CustomVisionTrainingClient(self.get_parameter('endpoint').value, credentials)

        if self.get_parameter('project_id').value is None:
            self.project = self.trainer.create_project(name=self.get_parameter('custom_vision_project_name').value,
                                                       classification_type='Multiclass')
            self.project_id = self.project.id
            
        else:
            self.project_id = self.get_parameter('project_id').value
            
            self.projects = self.trainer.get_projects()
            self.get_logger().info(str(self.projects))

    def stop_robot(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)  

    def _pose_callback(self, data):
        self.get_logger().info("New target pose recieved")
        self.target_pose = data.pose

    def get_robot_pose(self):

        # determine where the robot is on the map
        robot_map_position = self.tf_buffer.lookup_transform_full(
            target_frame='base_footprint',
            target_time=rclpy.time.Time(seconds=0),
            source_frame='map',
            source_time=rclpy.time.Time(seconds=0),
            fixed_frame='odom',
            timeout=rclpy.time.Duration(seconds=0.1))
        
        rot = Rotation.from_quat([
            robot_map_position.transform.rotation.x,
            robot_map_position.transform.rotation.y,
            robot_map_position.transform.rotation.z,
            robot_map_position.transform.rotation.w,
        ])

        euler = rot.as_euler('xyz', degrees=False)

        return robot_map_position, euler[0], euler[1], euler[2]
            
        

    def nav_and_photograph(self):
        if self.target_pose is None:
            self.get_logger().warn("No target pose set to navigate to!")
        try:
            robot_map_position, roll, pitch, yaw = self.get_robot_pose()
            y_diff =  self.target_pose.position.y - robot_map_position.transform.translation.y
            x_diff =  self.target_pose.position.x - robot_map_position.transform.translation.x
        
            rot = Rotation.from_quat([
                self.target_pose.orientation.x,
                self.target_pose.orientation.y,
                self.target_pose.orientation.z,
                self.target_pose.orientation.w,
            ])

            euler = rot.as_euler('xyz', degrees=False)
            target_yaw = euler[2]
            yaw_diff = target_yaw - yaw

            self.distance_to_goal = math.sqrt(y_diff ** 2 + x_diff ** 2)

            if self.distance_to_goal > 1.5 or self.state["nearest_scan_range"] < 0.4:
                self.command_with_policy()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                rotation_rate = -0.2

                self.get_logger().debug("---------------------------")
                self.get_logger().debug("Actual Orientation: {}".format(yaw))
                self.get_logger().debug("Goal Orientation: {}".format(target_yaw))
                self.get_logger().debug("Difference: {}".format(yaw_diff))
                self.get_logger().debug("---------------------------")

                if abs(yaw_diff) < 0.2:
                    self.stop_robot()
                    time.sleep(0.25)               
                    self.capture_and_upload()
                    self.command_with_policy()
                
                else:
                    msg.angular.z = rotation_rate * yaw_diff
                    self.cmd_vel_pub.publish(msg)

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform base_footprint to map: {ex}')
            self.command_with_policy()

    def update_state(self):
        if self.state_machine != self.get_parameter('state_machine_val').value:
            self.get_logger().info("New state recieved")

            # set the new state data
            self.state_machine = self.get_parameter('state_machine_val').value

            # create new tag every time the robot is commanded to take a new set of photos
            if self.state_machine == PHOTOGRAPHING:
                self.get_logger().info("Begining image captures")
                self.object_id = uuid.uuid4().hex
                self.tag_info = self.trainer.create_tag(self.project_id, self.object_id)
                self.get_logger().info(self.tag_info.id)
                self.image_num = 0

            elif self.state_machine == IDLE:
                self.stop_robot()

    def capture_and_upload(self):
        # take photos! 
        result, image = self.camera.read()
        self.image_num += 1

        if result:
            image_bytes = cv2.imencode('.jpg', image)[1].tobytes()
            self.trainer.create_images_from_data(self.project_id, image_bytes, tag_ids=[self.tag_info.id])
        else:
            self.get_logger().warn("Image capture failed")

        # Upload the photos to custom vision
        if self.image_num == MAX_UPLOAD_PER_TAG:
            self.get_logger().info("Image captures completed")
            param_int = Parameter('state_machine_val', Parameter.Type.INTEGER, IDLE)
            self.set_parameters([param_int])
            self.poi = None

    def run(self):
        while True:
            rclpy.spin_once(self)
            self.update_state()
            self.get_logger().debug("State is {}".format(self.state_machine))

            if self.state_machine == IDLE:
                time.sleep(1)

            elif self.state_machine == MAPPING:
                self.command_with_policy()

            elif self.state_machine == PHOTOGRAPHING:
                self.get_laser_scan_state_data()
                self.nav_and_photograph()

