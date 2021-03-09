#!/usr/bin/env python

from __future__ import print_function

from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import VehiclePositionRequest, VehiclePositionRequestResponse
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
    response.target_parking_position.longitude = 8.4202020245
    response.target_parking_position.latitude = 49.0141414141
    if req.info.loadable.load_during_parking:
        response.load_vehicle = True
    return response


def handle_request_vehicle_position(req):
    response = VehiclePositionRequestResponse()
    response.vehicle_status.status = 1
    response.position.longitude = 8.42011294615
    response.position.latitude = 49.01431771428
    return response


def start_server():
    rospy.init_node('service_server_capacity')
    service_capacity = rospy.Service('capacity_request', CapacityRequest, handle_request_capacity)
    service_parking = rospy.Service('register_vehicle_request', RegisterVehicleRequest, handle_request_register)
    service_position = rospy.Service('vehicle_position_request', VehiclePositionRequest, handle_request_vehicle_position)
    rospy.spin()


if __name__ == "__main__":
    start_server()


