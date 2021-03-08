from flask import jsonify
from datetime import datetime
import rospy
import threading
import enum

from std_msgs.msg import String
from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import LoadVehicleRequest, LoadVehicleRequestResponse
from parking_app_ros_pkg.msg import VehicleInformationMsg, VehicleLoadingMsg


class ChargeableType(enum.Enum):
    type_none = 0
    type_electric = 1
    type_electric_fast = 2
    type_electric_inductive = 3


class VehicleStatus(enum.Enum):
    status_transfer = 0
    status_parking_in = 1
    status_parked = 2
    status_parking_out = 3
    status_drop_off = 4
    status_picked_up = 5
    status_unknown = 6


class CommunicationRosServiceException(Exception):
    pass


class InternalCommunicationException(Exception):
    pass


ros_root_node = 'parking_node'
threading.Thread(target=lambda: rospy.init_node(ros_root_node, disable_signals=True)).start()


def retrieve_free_parking_spots_from_pms():
    try:
        rospy.wait_for_service('capacity_request', 5)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        capacity_request = rospy.ServiceProxy('capacity_request', CapacityRequest)
        capacities = capacity_request()
        return capacities
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))


def request_free_parking_spots(electric):
    current_capacities = retrieve_free_parking_spots_from_pms()
    if electric:
        free_electric = current_capacities.capacity_free.electric
        return jsonify({'free_electric': free_electric})
    else:
        free_total = current_capacities.capacity_free.total
        return jsonify({'free_total': free_total})


def request_capacities(free):
    current_capacities = retrieve_free_parking_spots_from_pms()
    if free:
        free_total = current_capacities.capacity_free.total
        free_electric = current_capacities.capacity_free.electric
        free_electric_fast = current_capacities.capacity_free.electric_fast
        free_electric_inductive = current_capacities.capacity_free.electric_inductive
        return jsonify({'free_total': free_total,
                        'free_electric': free_electric,
                        'free_electric_fast': free_electric_fast,
                        'free_electric_inductive': free_electric_inductive})
    else:
        total = current_capacities.capacity_total.total
        electric = current_capacities.capacity_total.electric
        electric_fast = current_capacities.capacity_total.electric_fast
        electric_inductive = current_capacities.capacity_total.electric_inductive
        return jsonify({'total': total,
                        'electric': electric,
                        'electric_fast': electric_fast,
                        'electric_inductive': electric_inductive})


def communicate_park_in(park_in_parameters):
    vehicle_message = generate_vehicle_message(park_in_parameters, vehicle_status=0)
    try:
        rospy.wait_for_service('register_vehicle_request', 5)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        register_vehicle_request = rospy.ServiceProxy('register_vehicle_request', RegisterVehicleRequest)
        response = register_vehicle_request(vehicle_message)
        return jsonify({'pms_id': response.pms_id})
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))


def generate_vehicle_message(park_in_parameters, vehicle_status):
    vehicle_message = VehicleInformationMsg()
    try:
        vehicle_message.identifiers.app_id = int(park_in_parameters["id"])
        vehicle_message.identifiers.number_plate = park_in_parameters["number_plate"]
        vehicle_message.dimensions.length = float(park_in_parameters["length"])
        vehicle_message.dimensions.width = float(park_in_parameters["width"])
        vehicle_message.dimensions.turning_radius = float(park_in_parameters["turning_radius"])
        vehicle_message.dimensions.dist_rear_axle_numberplate = float(park_in_parameters["dist_rear_axle_numberplate"])
    except KeyError as keyErrorReadingJSON:
        raise InternalCommunicationException(str(keyErrorReadingJSON))

    if "charge_type" in park_in_parameters:
        if park_in_parameters["charge_type"] == "electric":
            vehicle_message.type.type = ChargeableType.type_electric.value
        elif park_in_parameters["charge_type"] == "electric_fast":
            vehicle_message.type.type = ChargeableType.type_electric_fast.value
        elif park_in_parameters["charge_type"] == "electric_inductive":
            vehicle_message.type.type = ChargeableType.type_electric_inductive.value
    else:
        vehicle_message.type.type = ChargeableType.type_none.value

    if "near_exit" in park_in_parameters:
        if park_in_parameters["near_exit"] == True:
            vehicle_message.park_preferences.near_exit = 1
        else:
            vehicle_message.park_preferences.near_exit = 0

    if "parking_card" in park_in_parameters:
        if park_in_parameters["parking_card"] == True:
            vehicle_message.park_preferences.parking_card = 1
        else:
            vehicle_message.park_preferences.near_exit = 0
    vehicle_message.entry_time = rospy.get_rostime()
    vehicle_message.status.status = vehicle_status

    return vehicle_message


def communicate_load_vehicle(park_in_parameters):
    loading_message = generate_loading_message(park_in_parameters)
    try:
        rospy.wait_for_service('load_vehicle_request', 5)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        load_vehicle_request = rospy.ServiceProxy('load_vehicle_request', LoadVehicleRequest)
        response = load_vehicle_request(loading_message)
        return response.load_request_received
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))


def generate_loading_message(park_in_parameters):
    loading_message = VehicleLoadingMsg()
    try:
        loading_message.identifiers.app_id = int(park_in_parameters["id"])
        loading_message.identifiers.number_plate = park_in_parameters["number_plate"]

        if park_in_parameters["charge_type"] == "electric":
            loading_message.type.type = ChargeableType.type_electric.value
        elif park_in_parameters["charge_type"] == "electric_fast":
            loading_message.type.type = ChargeableType.type_electric_fast.value
        elif park_in_parameters["charge_type"] == "electric_inductive":
            loading_message.type.type = ChargeableType.type_electric_inductive.value
    except KeyError as keyErrorReadingJSON:
        raise InternalCommunicationException(str(keyErrorReadingJSON))

    if "state_of_charge" in park_in_parameters:
        loading_message.state_of_charge = int(park_in_parameters["state_of_charge"])
    if "charge_service_provider" in park_in_parameters:
        loading_message.preferred_charge_service_provider = park_in_parameters["charge_service_provider"]
    # TODO: Implement by resolving to rospy.time
    # if "charge_time_begin" in park_in_parameters:
    # if "charge_time_end" in park_in_parameters:

    return loading_message

