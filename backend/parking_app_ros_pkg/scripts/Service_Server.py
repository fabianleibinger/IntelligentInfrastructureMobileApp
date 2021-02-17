#!/usr/bin/env python

from __future__ import print_function

from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
import rospy


def handle_request_capacity(req):
    print('Handle')
    response = CapacityRequestResponse()
    response.capacity_total.total = 10
    response.capacity_free.total = 8
    response.capacity_total.electric = 5
    response.capacity_free.electric = 2
    return response


def request_capacity_server():
    rospy.init_node('service_server')
    s = rospy.Service('capacity_request', CapacityRequest, handle_request_capacity)
    print("Ready to request capacity.")
    rospy.spin()


if __name__ == "__main__":
    request_capacity_server()
    
