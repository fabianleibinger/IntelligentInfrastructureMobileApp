#!/usr/bin/env python

from __future__ import print_function

from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import VehiclePositionRequest, VehiclePositionRequestResponse
import rospy


# Script for running the ROS services (server-side)
# Services return dummy data for test purposes

def handle_request_capacity(req):
    """
    Generate a CapacityRequestResponse providing information about parking garage´s capacities.
    :param req: CapacityRequest
    :return: CapacityRequestResponse
    """
    response = CapacityRequestResponse()
    response.capacity_total.total = 10
    response.capacity_free.total = 8
    response.capacity_total.electric = 5
    response.capacity_free.electric = 2
    return response


def handle_request_register(req):
    """
    Generate a RegisterVehicleRequestResponse providing information about the vehicle status, the parking management
    system´s ID, the coordinates of the target parking position and whether the vehicle will be charged.
    :param req: RegisterVehicleRequest
    :return: RegisterVehicleRequestResponse
    """
    response = RegisterVehicleRequestResponse()
    response.vehicle_status.status = 1  # Parking in
    response.pms_id = 222
    response.target_parking_position.longitude = 8.4202020245
    response.target_parking_position.latitude = 49.0141414141
    if req.info.loadable.load_during_parking:
        response.load_vehicle = True
    return response


def handle_request_vehicle_position(req):
    """
    Generate a VehiclePositionRequestResponse providing information about the current vehicle position and its movement.
    :param req: VehiclePositionRequest
    :return: VehiclePositionRequestResponse
    """
    response = VehiclePositionRequestResponse()
    response.vehicle_status.status = 1
    response.position.longitude = 8.42011294615
    response.position.latitude = 49.01431771428
    return response


def start_server():
    """
    Initialize a ROS node and provide services from it.
    """
    rospy.init_node('service_server_capacity')
    service_capacity = rospy.Service('capacity_request', CapacityRequest, handle_request_capacity)
    service_parking = rospy.Service('register_vehicle_request', RegisterVehicleRequest, handle_request_register)
    service_position = rospy.Service('vehicle_position_request', VehiclePositionRequest, handle_request_vehicle_position)
    rospy.spin()


if __name__ == "__main__":
    # Entry point of the programme. The ROS service server will be setup.
    start_server()

