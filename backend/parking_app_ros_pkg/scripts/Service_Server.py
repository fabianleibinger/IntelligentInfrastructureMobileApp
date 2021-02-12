#!/usr/bin/env python

from __future__ import print_function

from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
import rospy


def handle_request_capacity(req):
    print('Handle')
    capacity_total = req.capacity.total
    return CapacityRequestResponse


def request_capacity_server():
    rospy.init_node('service_server')
    s = rospy.Service('capacity_request', CapacityRequest, handle_request_capacity)
    print("Ready to request capacity.")
    rospy.spin()


if __name__ == "__main__":
    request_capacity_server()
    
