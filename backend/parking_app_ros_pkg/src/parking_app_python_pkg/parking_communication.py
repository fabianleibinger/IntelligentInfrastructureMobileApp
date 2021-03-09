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
    """
    Vehicles can be differentiated by the charge type.
    The charge types should fit the corresponding ROS message VehicleTypeMsg.
    """
    type_none = 0
    type_electric = 1
    type_electric_fast = 2
    type_electric_inductive = 3


class VehicleStatus(enum.Enum):
    """
    The parking process can be divided in different phases. Each of them has a status.
    The vehicle status should fit the corresponding ROS message VehicleStatusMsg.
    """
    status_transfer = 0
    status_parking_in = 1
    status_parked = 2
    status_parking_out = 3
    status_drop_off = 4
    status_picked_up = 5
    status_unknown = 6


class CommunicationRosServiceException(Exception):
    """
    Unify exceptions occurring during communication with a ROS service with this custom exception.
    Catch this exception in flask server to provide an appropriate response to the frontend.
    """
    pass


class InternalCommunicationException(Exception):
    """
    Unify exceptions occurring due to different parameters or return types between flask server 
    and parking communication.
    Catch this exception in flask server to provide an appropriate response to the frontend.
    """
    pass


# Initialise a ROS node to allow ROS publisher/subscriber or requesting ROS services.
# Use threading to avoid collision with flask server.
ros_root_node = 'parking_node'
threading.Thread(target=lambda: rospy.init_node(ros_root_node, disable_signals=True)).start()
max_secs_to_wait_for_ros_service = 5


def retrieve_capacity_from_pms():
    """
    This method communicates with the parking management system and retrieves all capacities.
    Capacities are the total amount of parking spots in the parking garage as well as free parking spots.
    The data is divided both in total and free and normal and electric.
    :return: CapacityRequestResponse (ROS message)
    :exception: CommunicationRosServiceException if the ROS service is unavailable
    """
    try:
        rospy.wait_for_service('capacity_request', max_secs_to_wait_for_ros_service)
    except rospy.exceptions.ROSException as ros_exception:
        # If you get this exception here, probably your ROS service server is not running.
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        capacity_request = rospy.ServiceProxy('capacity_request', CapacityRequest)
        capacities = capacity_request()
        return capacities
    except rospy.ServiceException as service_exception:
        # If you get this exception here, probably your ROS service server run into an error processing the request.
        raise CommunicationRosServiceException(str(service_exception))


def request_free_parking_spots(electric: bool):
    """
    This method deals with the capacities returned from the parking management system.
    It provides an information about the currently free parking spots in the parking garage.
    :param electric: True if the request should be restricted to parking spots with possibility to charge
    :return: The currently free parking spots with specified traits as integer.
    """
    current_capacities = retrieve_capacity_from_pms()
    if electric:
        return current_capacities.capacity_free.electric
    else:
        return current_capacities.capacity_free.total


def request_capacities(free: bool):
    """
    This method deals with the capacities returned from the parking management system.
    It provides an information about all and not-occupied parking spots.
    :param free: True if the request should be restricted to not-occupied parking spots
    :return: A dictionary providing the capacity. Dictionary has keys: 'total', 'electric', 'electric_fast',
        'electric_inductive' (all integers) @see: CapacityMsg
    """
    current_capacities = retrieve_capacity_from_pms()
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
    """
    This method should be called when the flask server retrieves a park in request.
    From the given parameters, it generates a suitable ROS message and calls ROS service RegisterVehicleRequest.
    If the vehicle could be registered successfully, the parking management system chooses an appropriate parking spots
    and communicates its coordinates.
    :param park_in_parameters: A dictionary providing values which fit the VehicleInformationMsg
    :return: A park in response @see: generate_park_in_response()
    :exception: CommunicationRosServiceException if the ROS service is unavailable
    """
    vehicle_message = generate_vehicle_message(park_in_parameters, vehicle_status=0)
    try:
        rospy.wait_for_service('register_vehicle_request', max_secs_to_wait_for_ros_service)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        register_vehicle_request = rospy.ServiceProxy('register_vehicle_request', RegisterVehicleRequest)
        response = register_vehicle_request(vehicle_message)
        return generate_park_in_response(response, vehicle_message.identifiers.app_id)
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))


def generate_vehicle_message(park_in_parameters, vehicle_status):
    """
    This method generates a VehicleInformationMsg from the given parameters.
    There are necessary and optional parameters. If an optional parameter is not given, the value will be default.
    :param park_in_parameters: A dictionary which must have following keys:
        'id', 'number_plate', 'length', 'width', 'turning_radius', 'dist_rear_axle_numberplate'
        @see: VehicleIdentificationMsg & VehicleDimensionsMsg
        and can have following keys: 'charge_type', 'near_exit', 'parking_card'
        @see: ParkPreferencesMsg
    :param vehicle_status: @see: VehicleStatus
    :return: VehicleInformationMsg generated from the parameters
    :exception: InternatCommunicationException if a necessary parameter is missing.
    """
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
    # TODO: Work with appropriate time format
    vehicle_message.entry_time = rospy.get_rostime()
    vehicle_message.status.status = vehicle_status

    return vehicle_message


def generate_loading_message(park_in_parameters):
    """
    This method generates a VehicleLoadingMsg from the given parameters.
    A VehicleLoadingMsg provides information about if and how a vehicle should be charged during parking.
    :param park_in_parameters: A dictionary which must have key 'load' and optional keys 'state_of_charge',
        'charge_service_provider', 'charge_time_begin' and 'charge_time_end'. 'load' should be set to true if the
        vehicle should be charged during parking.
    :return: VehicleLoadingMsg generated from parameters
    """
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
    """
    This method generates a dictionary from the RegisterVehicleRequestResponse.
    If the vehicle was successfully registered and the parking management system started the park process,
    this method provides information about the target parking position and whether the vehicle will be charged.
    :param response_from_pms: RegisterVehicleRequestResponse
    :param app_id: the identification used in the parking app to identify this vehicle
    :return: A dictionary with keys 'parking_in', 'load_vehicle' (both boolean), 'longitude' and 'latitude'
        (both double, coordinates of the target parking position, NaN if registration failed).
    """
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
    """
    This method communicates with the parking management system to get the current position of the vehicle
    with this app id and number plate.
    From the given parameters, it generates a suitable ROS message and calls ROS service VehiclePositionRequest.
    If the vehicle could be located, it returns the current position as longitude and latitude
    as well as information about the parking process.
    :param app_id: the identification used in the parking app to identify this vehicle
    :param number_plate: the number plate/ license plate of the vehicle
    :return: A dictionary with keys 'longitude' and 'latitude' (both double)
        as well as 'moving' and 'reached_position' (both boolean)
    """
    vehicle_identification = VehicleIdentificationMsg()
    vehicle_identification.app_id = app_id
    vehicle_identification.pms_id = get_corresponding_pms_id(app_id)
    vehicle_identification.number_plate = number_plate

    try:
        rospy.wait_for_service('vehicle_position_request', max_secs_to_wait_for_ros_service)
    except rospy.exceptions.ROSException as ros_exception:
        raise CommunicationRosServiceException(str(ros_exception))
    try:
        vehicle_position_request = rospy.ServiceProxy('vehicle_position_request', VehiclePositionRequest)
        response = vehicle_position_request(vehicle_identification)

        if response.vehicle_status.status == VehicleStatus.status_parking_in.value\
                or response.vehicle_status.status == VehicleStatus.status_parking_out.value:
            in_park_process = True
        else:
            in_park_process = False

        if response.vehicle_status.status == VehicleStatus.status_parked.value\
                or response.vehicle_status.status == VehicleStatus.status_drop_off.value:
            reached_target_position = True
        else:
            reached_target_position = False

        return {'longitude': response.position.longitude,
                'latitude': response.position.latitude,
                'moving': in_park_process,
                'reached_position': reached_target_position}
    except rospy.ServiceException as service_exception:
        raise CommunicationRosServiceException(str(service_exception))

