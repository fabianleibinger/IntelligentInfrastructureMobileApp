import datetime
import rospy
import threading
import enum
import parking_app_python_pkg.database as id_database

from std_msgs.msg import String
from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse
from parking_app_ros_pkg.srv import VehiclePositionRequest, VehiclePositionRequestResponse
from parking_app_ros_pkg.srv import ParkoutVehicleRequest, ParkoutVehicleRequestResponse
from parking_app_ros_pkg.msg import VehicleInformationMsg, VehicleLoadingMsg, VehicleIdentificationMsg

# Initialise a ROS node to allow ROS publisher/subscriber or requesting ROS services.
# Use threading to avoid collision with flask server.
ros_root_node = 'parking_node'
threading.Thread(target=lambda: rospy.init_node(ros_root_node, disable_signals=True)).start()
max_secs_to_wait_for_ros_service = 5
unknown_pms_id = -1


def communicate_park_in(park_in_parameters: dict):
    """
    This function serves as an interface for the flask server.
    It delegates the park in process. It will try to register the vehicle to the parking garage´s system.
    If successful, it returns how the park process is going and the coordinates of the target parking position.
    :param park_in_parameters: A dictionary representing the vehicle´s traits
    :return: A dictionary with information about the park process and the target park position´s coordinates
    """
    park_process_helper = ParkProcess()
    return park_process_helper.register_vehicle_to_pms(park_in_parameters)


def communicate_park_out(app_id: int, number_plate: str):
    """
    This function serves as an interface for the flask server.
    It delegates the park out process. It will try to park out the vehicle from the parking garage.
    If successful, it returns how the park process is going and the coordinates of the drop-off zone.
    :param app_id: ID used in parking app to identify vehicle
    :param number_plate: Official number plate
    :return: A dictionary with information about the park process and the transfer zone´s coordinates.
    """
    park_process_helper = ParkProcess()
    return park_process_helper.park_out_vehicle_from_pms(app_id, number_plate)


def communicate_free_parking_spots(electric: bool):
    """
    This function serves as an interface for the flask server.
    It delegates the capacity request. It requests the currently free parking spots from the parking management system
    and returns them.
    :param electric: True if the request is limited to parking spots with electric charging.
    :return: The free parking spots fitting the restrictions
    """
    capacity_process_helper = CapacityProcess()
    return capacity_process_helper.request_free_parking_spots(electric)


def communicate_capacities(free: bool):
    """
    This function serves as an interface for the flask server.
    It delegates the capacity request. It requests a summary about the parking garage´s capacities.
    :param free: True if the request is limited to unoccupied parking spots.
    :return: A dictionary providing information about the capacities
    """
    capacity_process_helper = CapacityProcess()
    return capacity_process_helper.request_capacities(free)


def communicate_vehicle_position(app_id: int, number_plate: str):
    """
    This function serves as an interface for the flask server.
    It delegates the localization process. It requests the current position of the vehicle with the given identifiers.
    :param app_id: ID used in parking app to identify vehicle
    :param number_plate: Official number plate
    :return: A dictionary with the coordinates of the vehicle and whether it is moving or parked.
    """
    localization_process_helper = LocalizationProcess()
    return localization_process_helper.request_current_position(app_id, number_plate)


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
    status_pickup = 5
    status_unknown = 6


class ParkProcess:
    def register_vehicle_to_pms(self, vehicle_parameters):
        """
            This method should be called when the flask server retrieves a park in request.
            From the given parameters, it generates a suitable ROS message and calls ROS service RegisterVehicleRequest.
            If the vehicle could be registered successfully, the parking management system chooses an appropriate
            parking spot (for park in) and communicates its coordinates.
            :param vehicle_parameters: A dictionary providing values which fit the VehicleInformationMsg
            :return: A parking response @see: generate_park_in_response()
            :exception: CommunicationRosServiceException if the ROS service is unavailable
            """
        vehicle_message = self.generate_vehicle_message(
            vehicle_parameters, vehicle_status=VehicleStatus.status_transfer.value)
        vehicle_message.entry_time = rospy.get_rostime()
        try:
            rospy.wait_for_service('register_vehicle_request', max_secs_to_wait_for_ros_service)
        except rospy.exceptions.ROSException as ros_exception:
            raise CommunicationRosServiceException(str(ros_exception))
        try:
            register_vehicle_request = rospy.ServiceProxy('register_vehicle_request', RegisterVehicleRequest)
            response = register_vehicle_request(vehicle_message)
            return self.generate_park_in_response(response, vehicle_message.identifiers.app_id)
        except rospy.ServiceException as service_exception:
            raise CommunicationRosServiceException(str(service_exception))

    def park_out_vehicle_from_pms(self, app_id: int, number_plate: str):
        """
        This method should be called when the flask server retrieves a park out request.
        From the given parameters, it generates a suitable ROS message and calls ROS service ParkoutVehicleRequest.
        If the vehicle could be identified, the parking management system induces a park out and communicates the
        coordinates of the transfer zone.
        :param app_id: ID used in parking app to identify vehicle
        :param number_plate: Official number plate
        :return: A parking response @see: generate_park_out_response
        """
        pms_id = IdMapper.get_corresponding_pms_id(app_id)
        vehicle_identification_message = self.generate_identification_message(app_id, number_plate, pms_id)
        try:
            rospy.wait_for_service('parkout_vehicle_request', max_secs_to_wait_for_ros_service)
        except rospy.exceptions.ROSException as ros_exception:
            raise CommunicationRosServiceException(str(ros_exception))
        try:
            parkout_vehicle_request = rospy.ServiceProxy('parkout_vehicle_request', ParkoutVehicleRequest)
            response = parkout_vehicle_request(vehicle_identification_message)
            return self.generate_park_out_response(response)
        except rospy.ServiceException as service_exception:
            raise CommunicationRosServiceException(str(service_exception))

    def generate_vehicle_message(self, vehicle_parameters, vehicle_status):
        """
        This method generates a VehicleInformationMsg from the given parameters.
        There are necessary and optional parameters. If an optional parameter is not given, the value will be default.
        :param vehicle_parameters: A dictionary which must have following keys:
            'id', 'number_plate', 'length', 'width', 'turning_radius', 'dist_rear_axle_numberplate'
            @see: VehicleIdentificationMsg & VehicleDimensionsMsg
            and can have following keys: 'charge_type', 'near_exit', 'parking_card'
            @see: ParkPreferencesMsg
        :param vehicle_status: @see: VehicleStatus
        :return: VehicleInformationMsg generated from the parameters
        :exception: InternalCommunicationException if a necessary parameter is missing.
        """
        vehicle_message = VehicleInformationMsg()
        try:
            vehicle_message.identifiers = self.generate_identification_message(
                int(vehicle_parameters["id"]), vehicle_parameters["number_plate"])
            vehicle_message.dimensions.length = float(vehicle_parameters["length"])
            vehicle_message.dimensions.width = float(vehicle_parameters["width"])
            vehicle_message.dimensions.turning_radius = float(vehicle_parameters["turning_radius"])
            vehicle_message.dimensions.dist_rear_axle_numberplate = float(
                vehicle_parameters["dist_rear_axle_numberplate"])
        except KeyError as keyErrorReadingJSON:
            raise InternalCommunicationException(str(keyErrorReadingJSON))

        vehicle_message = self.add_optional_vehicle_parameters(vehicle_parameters, vehicle_message)
        vehicle_message.status.status = vehicle_status

        return vehicle_message

    def generate_identification_message(self, app_id: int, number_plate: str, pms_id: int = unknown_pms_id):
        """
        This method generates a vehicle identification message.
        This message includes the app ID and the number plate as well as the corresponding parking management system´s
        ID if the vehicle has been registered. Otherwise, the parking management system´s ID is unknown_pms_id.
        :param pms_id: ID used in parking management system to identify vehicle
        :param app_id: ID used in parking app to identify vehicle
        :param number_plate: Official number plate
        :return: VehicleIdentificationMsg from the parameters
        """
        vehicle_identification_message = VehicleIdentificationMsg()
        vehicle_identification_message.app_id = app_id
        vehicle_identification_message.number_plate = number_plate
        vehicle_identification_message.pms_id = pms_id
        return vehicle_identification_message

    def add_optional_vehicle_parameters(self, vehicle_parameters, vehicle_message: VehicleInformationMsg):
        """
        This method adds the values from the optional vehicle parameters to the given vehicle message.
        :param vehicle_parameters: dictionary with keys 'charge_type', 'near_exit', 'parking_card'
        :param vehicle_message: The VehicleInformationMsg which should be modified
        :return: VehicleInformationMsg as a combination of param vehicle_message and values from vehicle_parameters
        """
        modified_vehicle_message = vehicle_message
        if "charge_type" in vehicle_parameters:
            if vehicle_parameters["charge_type"] == "electric":
                modified_vehicle_message.type.type = ChargeableType.type_electric.value
            elif vehicle_parameters["charge_type"] == "electric_fast":
                modified_vehicle_message.type.type = ChargeableType.type_electric_fast.value
            elif vehicle_parameters["charge_type"] == "electric_inductive":
                modified_vehicle_message.type.type = ChargeableType.type_electric_inductive.value
        else:
            modified_vehicle_message.type.type = ChargeableType.type_none.value

        if modified_vehicle_message.type.type != 0 and "load" in vehicle_parameters:
            loading_message = self.generate_loading_message(vehicle_parameters)
            modified_vehicle_message.loadable = loading_message

        if "near_exit" in vehicle_parameters:
            if vehicle_parameters["near_exit"] == True:
                modified_vehicle_message.park_preferences.near_exit = 1

        if "parking_card" in vehicle_parameters:
            if vehicle_parameters["parking_card"] == True:
                modified_vehicle_message.park_preferences.parking_card = 1

        return modified_vehicle_message

    def generate_loading_message(self, park_in_parameters):
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

        charge_time_delimiter = ":"
        if "charge_time_begin" in park_in_parameters and "charge_time_end" in park_in_parameters:
            begin_hour_min = park_in_parameters["charge_time_begin"].split(charge_time_delimiter)
            end_hour_min = park_in_parameters["charge_time_end"].split(charge_time_delimiter)
            time_span = self.generate_charge_time_span(int(begin_hour_min[0]), int(begin_hour_min[1]),
                                                       int(end_hour_min[0]), int(end_hour_min[1]))
            loading_message.preferred_charge_time_begin = rospy.Time.from_sec(
                time_span[0].replace(tzinfo=datetime.timezone.utc).timestamp())
            loading_message.preferred_charge_time_end = rospy.Time.from_sec(
                time_span[1].replace(tzinfo=datetime.timezone.utc).timestamp())
        return loading_message

    def generate_charge_time_span(self, start_hour: int, start_minute: int, end_hour: int, end_minute: int):
        """
        This method generates a time span which modifies the vehicle´s charge process.
        The time span will be the next clock hands´ stand with the borders of start and end.
        The time span will be today or tomorrow.
        :param start_hour: Time span not before HH:MM, HH in [0,24]
        :param start_minute: Time span not before HH:MM, MM in [0,59]
        :param end_hour: Time span not after HH:MM, HH in [0,24]
        :param end_minute: Time span not after HH:MM, MM in [0,59]
        :return: A list with first entry begin (type: time) and second entry end (type: time)
        """
        if start_hour > 24 or end_hour > 24 or start_minute > 59 or end_minute > 59 or \
                (start_hour == 24 and start_minute > 0) or (end_hour == 24 and end_minute > 0):
            raise KeyError("Not an appropriate time format for HH:MM.")

        start_time = datetime.time(hour=start_hour, minute=start_minute)
        end_time = datetime.time(hour=end_hour, minute=end_minute)
        today = datetime.datetime.today()
        tomorrow = datetime.datetime.today() + datetime.timedelta(days=1)
        current_time = datetime.time(hour=today.hour, minute=today.minute)

        # if time span is unlimited (00:00-24:00)
        if start_hour == 0 and start_minute == 0 and end_hour == 24 and end_minute == 0:
            begin = datetime.datetime.combine(today, current_time)
            end = datetime.datetime.combine(tomorrow, current_time)
            return [begin, end]

        # next time span fitting the limits of start and end time
        if start_time <= end_time:  # start_time and end_time on same day
            if current_time <= end_time:  # time span can be today
                end = datetime.datetime.combine(today, end_time)
                if current_time <= start_time:
                    begin = datetime.datetime.combine(today, start_time)
                else:
                    begin = datetime.datetime.combine(today, current_time)
            else:
                begin = datetime.datetime.combine(tomorrow, start_time)
                end = datetime.datetime.combine(tomorrow, end_time)
        else:  # end_time refers to a time of tomorrow
            if current_time <= end_time:
                begin = datetime.datetime.combine(today, current_time)
                end = datetime.datetime.combine(today, end_time)
            else:
                end = datetime.datetime.combine(tomorrow, end_time)
                if current_time <= start_time:
                    begin = datetime.datetime.combine(today, start_time)

                else:
                    begin = datetime.datetime.combine(today, current_time)
        return [begin, end]

    def generate_park_in_response(self, response_from_pms: RegisterVehicleRequestResponse, app_id: int):
        """
        This method generates a dictionary from the RegisterVehicleRequestResponse.
        If the vehicle was successfully registered and the parking management system started the park process,
        this method provides information about the target parking or transfer position and whether the vehicle will be
        charged.
        :param response_from_pms: RegisterVehicleRequestResponse
        :param app_id: the identification used in the parking app to identify this vehicle
        :return: A dictionary with keys 'parking', 'load_vehicle' (both boolean), 'longitude' and 'latitude'
            (both double, coordinates of the target parking position if park in process or of the transfer zone if park
            out process, NaN if registration failed).
        """
        if response_from_pms.vehicle_status.status == VehicleStatus.status_transfer.value \
                or response_from_pms.vehicle_status.status == VehicleStatus.status_parking_in.value:
            IdMapper.map_vehicle_ids(app_id, response_from_pms.pms_id)
            return {'parking_in': True,
                    'longitude': response_from_pms.target_parking_position.longitude,
                    'latitude': response_from_pms.target_parking_position.latitude,
                    'load_vehicle': response_from_pms.load_vehicle}
        else:
            return {'parking_in': False,
                    'longitude': float('NaN'),
                    'latitude': float('NaN'),
                    'load_vehicle': False}

    def generate_park_out_response(self, response_from_pms: ParkoutVehicleRequestResponse):
        if response_from_pms.vehicle_status.status == VehicleStatus.status_parking_out.value \
                or response_from_pms.vehicle_status.status == VehicleStatus.status_drop_off.value:
            return {'parking_out': True,
                    'longitude': response_from_pms.transfer_zone.longitude,
                    'latitude': response_from_pms.transfer_zone.latitude}
        else:
            return {'parking_out': False,
                    'longitude': float('NaN'),
                    'latitude': float('NaN')}


class LocalizationProcess:
    def request_current_position(self, app_id: int, number_plate: str):
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
        park_process_ids = ParkProcess()
        pms_id = IdMapper.get_corresponding_pms_id(app_id)
        vehicle_identification = park_process_ids.generate_identification_message(app_id, number_plate, pms_id)

        try:
            rospy.wait_for_service('vehicle_position_request', max_secs_to_wait_for_ros_service)
        except rospy.exceptions.ROSException as ros_exception:
            raise CommunicationRosServiceException(str(ros_exception))
        try:
            vehicle_position_request = rospy.ServiceProxy('vehicle_position_request', VehiclePositionRequest)
            response = vehicle_position_request(vehicle_identification)
            return self.generate_get_position_response(response)
        except rospy.ServiceException as service_exception:
            raise CommunicationRosServiceException(str(service_exception))

    def generate_get_position_response(self, response: VehiclePositionRequestResponse):
        if response.vehicle_status.status == VehicleStatus.status_transfer \
                or response.vehicle_status.status == VehicleStatus.status_parking_in.value \
                or response.vehicle_status.status == VehicleStatus.status_parking_out.value \
                or response.vehicle_status.status == VehicleStatus.status_drop_off.value:
            in_park_process = True
        else:
            in_park_process = False

        if response.vehicle_status.status == VehicleStatus.status_parked.value \
                or response.vehicle_status.status == VehicleStatus.status_pickup.value:
            reached_target_position = True
        else:
            reached_target_position = False

        return {'longitude': response.position.longitude,
                'latitude': response.position.latitude,
                'moving': in_park_process,
                'reached_position': reached_target_position}
        

class CapacityProcess:
    def retrieve_capacity_from_pms(self):
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

    def request_free_parking_spots(self, electric: bool):
        """
        This method deals with the capacities returned from the parking management system.
        It provides an information about the currently free parking spots in the parking garage.
        :param electric: True if the request should be restricted to parking spots with possibility to charge
        :return: The currently free parking spots with specified traits as integer.
        """
        current_capacities = self.retrieve_capacity_from_pms()
        if electric:
            return current_capacities.capacity_free.electric
        else:
            return current_capacities.capacity_free.total

    def request_capacities(self, free: bool):
        """
        This method deals with the capacities returned from the parking management system.
        It provides an information about all and not-occupied parking spots.
        :param free: True if the request should be restricted to not-occupied parking spots
        :return: A dictionary providing the capacity. Dictionary has keys: 'total', 'electric', 'electric_fast',
            'electric_inductive' (all integers) @see: CapacityMsg
        """
        current_capacities = self.retrieve_capacity_from_pms()
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


class IdMapper:
    @staticmethod
    def map_vehicle_ids(app_id: int, pms_id: int):
        """
        This method adds the pair of app id and parking management system id to the database.
        In the database, IDs can be stored permanently. IDs are used to identify vehicles uniquely.
        :param app_id: The key used for a vehicle in parking app
        :param pms_id: The key used for a vehicle in parking management system
        """
        id_database.add(app_id, pms_id)
        id_database.add(app_id, pms_id)

    @staticmethod
    def get_corresponding_pms_id(app_id: int):
        """
        This method returns the corresponding parking management system´s ID to the given app id.
        The value will be requested from the id mapping database.
        :param app_id: The key used for a vehicle in parking app
        :return: Parking management system´s ID as integer
        :exception: VehicleIdentificationException if no vehicle with this app_id is registered
        """
        pms_id = id_database.get_parking_garage_id(app_id)
        if pms_id is None:
            raise VehicleIdentificationException(
                'Database could not find a corresponding ID in parking garage´s system. Vehicle ist not registered.')
        else:
            return int(pms_id)
        

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


class VehicleIdentificationException(Exception):
    """
    Unify exceptions occurring due to unknown app identifiers in the ID mapping database.
    """
    pass

