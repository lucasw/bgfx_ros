#!/usr/bin/env python
# Copyright 2017 Lucas Walter
#
# Generate triangle list Marker

import math
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def sphere_point(radius, theta, phi):
    pt = Point()
    sp = math.sin(phi)
    cp = math.cos(phi)
    pt.x = radius * cp * math.cos(theta)
    pt.y = radius * cp * math.sin(theta)
    pt.z = radius * sp
    return pt

rospy.init_node('generate_marker')

pub = rospy.Publisher("marker", Marker, queue_size=2)

marker = Marker()
marker.header.frame_id = rospy.get_param("~frame_id", "frame1")
marker.ns = marker.header.frame_id
marker.id = 0
marker.type = Marker.TRIANGLE_LIST
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0

marker.color.a = 1.0
marker.color.r = rospy.get_param("~r", 1.0)
marker.color.g = rospy.get_param("~g", 1.0)
marker.color.b = rospy.get_param("~b", 1.0)

radius = rospy.get_param("~radius", 2.0)
num_lat = 20
num_long = 20
for i in range(num_lat):
    phi1 = float(i) / float(num_lat) * math.pi - math.pi / 2.0
    phi2 = float(i + 1) / float(num_lat) * math.pi - math.pi / 2.0
    for j in range(num_long):
        theta1 = float(j) / float(num_long) * 2.0 * math.pi
        theta2 = float(j + 1) / float(num_long) * 2.0 * math.pi

        c11 = ColorRGBA()
        c11.a = 1.0
        c11.r = (j / 10.0) % 1.0
        c11.g = (i / 20.0) % 1.0
        c11.b = (j / 40.0) % 1.0

        p11 = sphere_point(radius, theta1, phi1)
        p12 = sphere_point(radius, theta1, phi2)
        p21 = sphere_point(radius, theta2, phi1)
        p22 = sphere_point(radius, theta2, phi2)

        face1 = True
        if i == 0:
            face1 = False
        if face1:
            marker.points.append(p11)
            marker.points.append(p12)
            marker.points.append(p21)

            marker.colors.append(c11)
            marker.colors.append(c11)
            marker.colors.append(c11)

        face2 = True
        if i == num_lat - 1:
            face2 = False

        if face2:
            marker.points.append(p21)
            marker.points.append(p12)
            marker.points.append(p22)

            marker.colors.append(c11)
            marker.colors.append(c11)
            marker.colors.append(c11)

while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    rospy.sleep(0.5)
