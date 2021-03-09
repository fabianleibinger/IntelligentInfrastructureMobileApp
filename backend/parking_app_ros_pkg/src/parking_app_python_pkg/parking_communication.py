from datetime import datetime
import rospy
import threading
import enum

from std_msgs.msg import String
from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import VehiclePositionRequest, VehiclePositionRequestResponse
from parking_app_ros_pkg.msg import VehicleInformationMsg, VehicleLoadingMsg, VehicleIdentificationMsg


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
        return {'electric': free_electric}
    else:
        free_total = current_capacities.capacity_free.total
        return {'total': free_total}


def request_capacities(free):
    current_capacities = retrieve_free_parking_spots_from_pms()
    if free:
        free_total = current_capacities.capacity_free.total
        free_electric = current_capacities.capacity_free.electric
        free_electric_fast = current_capacities.capacity_free.electric_fast
        free_electric_inductive = current_capacities.capacity_free.electric_inductive
        return {'total': free_total,
                'electric': free_electric,
                'electric_fast': free_electric_fast,
                'electric_inductive': free_electric_inductive}
    else:
        total = current_capacities.capacity_total.total
        electric = current_capacities.capacity_total.electric
        electric_fast = current_capacities.capacity_total.electric_fast
        electric_inductive = current_capacities.capacity_total.electric_inductive
        return {'total': total,
                'electric': electric,
                'electric_fast': electric_fast,
                'electric_inductive': electric_inductive}


def communicate_park_in(park_in_parameters):
    vehicle_message = generate_vehicle_message(park_in_parameters, vehicle_status=0)
    try:
        rospy.wait_for_service('register_vehicle_request', 5)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        register_vehicle_request = rospy.ServiceProxy('register_vehicle_request', RegisterVehicleRequest)
        response = register_vehicle_request(vehicle_message)
        return generate_park_in_response(response, vehicle_message.identifiers.app_id)
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

    if vehicle_message.type.type != 0 and "load" in park_in_parameters:
        loading_message = generate_loading_message(park_in_parameters)
        vehicle_message.loadable = loading_message

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


def generate_loading_message(park_in_parameters):
    loading_message = VehicleLoadingMsg()

    loading_message.load_during_parking = park_in_parameters["load"]

    if "state_of_charge" in park_in_parameters:
        loading_message.state_of_charge = int(park_in_parameters["state_of_charge"])
    if "charge_service_provider" in park_in_parameters:
        loading_message.preferred_charge_service_provider = park_in_parameters["charge_service_provider"]
    # TODO: Implement by resolving to rospy.time
    # if "charge_time_begin" in park_in_parameters:
    # if "charge_time_end" in park_in_parameters:

    return loading_message


def generate_park_in_response(response_from_pms, app_id):
    if response_from_pms.vehicle_status.status == VehicleStatus.status_parking_in.value:
        map_vehicle_ids(app_id, response_from_pms.pms_id)
        return {'parking_in': True,
                'longitude': response_from_pms.target_parking_position.longitude,
                'latitude': response_from_pms.target_parking_position.latitude,
                'load_vehicle': response_from_pms.load_vehicle}
    else:
        return {'parking_in': False,
                'longitude': float('NaN'),
                'latitude': float('NaN'),
                'load_vehicle': False}


def map_vehicle_ids(app_id, pms_id):
    # TODO: save IDs in database
    return


def get_corresponding_pms_id(app_id):
    # TODO: get ID from database
    return 12


def request_current_position(app_id, number_plate):
    vehicle_identification = VehicleIdentificationMsg()
    vehicle_identification.app_id = app_id
    vehicle_identification.pms_id = get_corresponding_pms_id(app_id)
    vehicle_identification.number_plate = number_plate

    try:
        rospy.wait_for_service('vehicle_position_request', 5)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        vehicle_position_request = rospy.ServiceProxy('vehicle_position_request', VehiclePositionRequest)
        response = vehicle_position_request(vehicle_identification)

        if response.vehicle_status.status == VehicleStatus.status_parking_in.value\
                or response.vehicle_status.status == VehicleStatus.status_parking_out:
            in_park_process = True
        else:
            in_park_process = False
        if response.vehicle_status.status == VehicleStatus.status_parked.value\
                or response.vehicle_status.status == VehicleStatus.status_drop_off:
            reached_target_position = True
        else:
            reached_target_position = False

        return {'longitude': response.position.longitude,
                'latitude': response.position.latitude,
                'moving': in_park_process,
                'reached_position': reached_target_position}
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))

