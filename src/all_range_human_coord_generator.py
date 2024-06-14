#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import roslib
import yaml
import rosparam
from geometry_msgs.msg import Twist
from happymimi_msgs.srv import StrToStr, StrToStrRequest, SimpleTrg
from all_range_human.srv import AllRange
from darknet_ros_msgs.msg import BoundingBoxes

file_config_path = roslib.packages.get_pkg_dir('happymimi_params') + '/location/tmp_human_location.yaml'
param_path = roslib.packages.get_pkg_dir("happymimi_params")

class MilkyCtrMegarover():
    def __init__(self):
        rospy.Service('/all_range_human', AllRange, self.allcallback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.twist = Twist()
        self.coord_gen_srv = rospy.ServiceProxy('/human_coord_generator', StrToStr)
        rospy.Subscriber('/darknet_ros/YOLO_BoundingBoxes', BoundingBoxes, self.callback)
        self.center_x = 320
        self.center_y = 240
        self.human_location_dict = {}
        
    def callback(self, bbox):
        self.xmin = bbox.xmin
        self.xmax = bbox.xmax
        self.ymin = bbox.ymin
        self.ymax = bbox.ymax
        bbox_center_x = (self.xmax + self.xmin) / 2
        bbox_center_y = (self.ymax + self.ymin) / 2
        self.diff_x = abs(self.center_x - bbox_center_x)
        self.diff_y = abs(self.center_y - bbox_center_y)
        
    def allcallback(self, num):
        self.number = num

    def main(self):
        rospy.loginfo("start generator")
        self.twist.angular.z = 0.2
        self.vel_pub.publish(self.twist.angular.z)
        for i in range(0, self.number):
            key = "human_" + i
            if self.diff_x <= 30:
                srv = StrToStrRequest()
                srv.req_data = "normal"
                #self.base_control.rotateAngle(180, 0, 1.0, 10)
                #人接近 = 場所にいる人物に近づく
                rospy.set_param("/map_range/min_x", -3.5)
                rospy.set_param("/map_range/max_x", 3.0)
                rospy.set_param("/map_range/min_y", -3.0)
                rospy.set_param("/map_range/max_y", 10.0)
                rospy.sleep(1.5)
                self.head_pose.publish(-15.0)
                rospy.sleep(1.5)
                try:
                    result = self.coord_gen_srv(srv).result
                    print('coord',result)
                except Exception as e:
                    print(e)
                    pass
                with open(file_config_path, 'r') as file:
                    yaml_data = yaml.safe_load(file)
                if yaml_data:
                    self.human_location_dict[key] = yaml_data['human_0']
                    # rospy.set_param('/all_range_human_location', self.human_location)
                    # rosparam.dump_params(param_path + '/location/'  + 'all_range_human_location.yaml', '/all_range_human_location')
        with open(param_path + '/location/'  + 'all_range_human_location.yaml', 'w+') as file:
            file.truncate(0)
            yaml.dump(self.human_location_dict, file, encoding='utf-8', allow_unicode=True)
        self.human_location_dict = {}
        return "True"
                
        
        
