#!/usr/bin/env python

from __future__ import print_function

from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import LoadVehicleRequest, LoadVehicleRequestResponse
import rospy


def handle_request_capacity(req):
    response = CapacityRequestResponse()
    response.capacity_total.total = 10
    response.capacity_free.total = 8
    response.capacity_total.electric = 5
    response.capacity_free.electric = 2
    return response


def handle_request_register(req):
    response = RegisterVehicleRequestResponse()
    response.vehicle_status.status = 1  # Parking in
    response.pms_id = 222
    return response


def handle_request_load(req):
    response = LoadVehicleRequestResponse()
    response.load_request_received = True
    return response


def start_server():
    rospy.init_node('service_server_capacity')
    service_capacity = rospy.Service('capacity_request', CapacityRequest, handle_request_capacity)
    service_parking = rospy.Service('register_vehicle_request', RegisterVehicleRequest, handle_request_register)
    service_loading = rospy.Service('load_vehicle_request', LoadVehicleRequest, handle_request_load)
    rospy.spin()


if __name__ == "__main__":
    start_server()


