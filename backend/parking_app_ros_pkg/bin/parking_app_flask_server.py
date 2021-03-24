import os
import math
import configparser
from flask import Flask, jsonify, request, Response, redirect

# database python module provides the logic for the IDMapping
# for app_id and pms_id
import parking_app_python_pkg.database as id_mapping
from parking_app_python_pkg.database import db as database_vehicle_ids

# parking_communication python module provides the logic for
# communication between flask server and ROS nodes
import parking_app_python_pkg.parking_communication as communication

############################################################################
# Start Flask server with configuration from config.properties file
app = Flask(__name__)

dir_path = os.path.dirname(os.path.realpath(__file__))
config = configparser.RawConfigParser()
# Save configuration file in same folder and check name matching
configFilePath = os.path.join(dir_path, 'config.properties')
config.read(configFilePath)

# Load configuration from configuration file for url and port
url_address = config['server']['ipAddress']
port = config.getint('server', 'port')

# Load parking garage name from configuration file
parking_garage_name = config['parking garage']['name']

############################################################################

# Defines the place of the database and init the connection between app and database
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///site.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
database_vehicle_ids.init_app(app)
app.app_context().push()

############################################################################

# This message should be sent to the client when the ROS service did not return a valid value.
communication_failed_message = \
    "The parking garage management system could not return a valid response or is unavailable."


############################################################################


# HTTP routes: access the server by calling http://<IP>:<Port>/<route>
# Use JSON for sending data.

@app.route('/')
def main():
    """
    Main HTTP route. Communicates the server´s purpose.
    :return: Welcome message
    """
    return "This is a parking garage´s server."


@app.route('/connect')
def return_connection_information():
    """
    HTTP route for connection check.
    This route provides information about the parking garage´s name listed under this IP and port.
    :return: HTTP response with JSON with fields 'IP' (String), 'Port' (Int) and 'parking_garage' (String).
    """
    return jsonify({'IP': url_address, 'Port': port, 'parking_garage': parking_garage_name})


@app.route('/capacities', methods=['GET', 'POST'])
def get_capacities():
    """
    Capacities are all parking spots in a parking garage.
    They can be divided into parking spots with or without charge columns for electric charging.
    :return: HTTP response with status code 200 and JSON fields 'total', 'electric', 'electric_fast' and
        'electric_inductive' (all integers)
        or HTTP response with status code 503 if the service did not return a valid response.
    """
    try:
        capacities = communication.communicate_capacities(False)
        return jsonify({'total': capacities["total"],
                        'electric': capacities["electric"],
                        'electric_fast': capacities["electric_fast"],
                        'electric_inductive': capacities["electric_inductive"]})
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free', methods=['GET', 'POST'])
def get_free_capacities():
    """
    Free capacities are all not occupied parking spots in a parking garage.
    They can be divided into parking spots with or without charge columns for electric charging.
    :return: HTTP response with status code 200 and JSON fields 'free_total', 'free_electric', 'free_electric_fast' and
        'free_electric_inductive' (all integers)
        or HTTP response with status code 503 if the service did not return a valid response.
    """
    try:
        capacities = communication.communicate_capacities(True)
        return jsonify({'free_total': capacities["total"],
                        'free_electric': capacities["electric"],
                        'free_electric_fast': capacities["electric_fast"],
                        'free_electric_inductive': capacities["electric_inductive"]})
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free/total', methods=['GET', 'POST'])
def all_free_parking_spots():
    """
    Request the currently free parking spots in the parking garage.
    :return: HTTP response with status code 200 and JSON field 'free_total' providing the currently free parking spots
        as integer value
        or HTTP response with status code 503 if the service did not return a valid response.
    """
    try:
        capacity = communication.communicate_free_parking_spots(False)
        return jsonify({'free_total': capacity})
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free/electric', methods=['GET', 'POST'])
def electric_free_parking_spots():
    """
    Request the currently free parking spots in the parking garage which have the possibility of electric charging.
    :return: HTTP response with status code 200 and JSON field 'free_electric' providing the currently free parking
        spots with electric charging as integer value
        or HTTP response with status code 503 if the service did not return a valid response.
    """
    try:
        capacity = communication.communicate_free_parking_spots(True)
        return jsonify({'free_electric': capacity})
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/parkIn', methods=['POST'])
def perform_park_in_request():
    """
    This route is called when an app user wants to park his vehicle into the parking garage.
    The request must provide information about the vehicle to allow an identification.
    After the park in request, the vehicle will be registered to the parking management system and the parking
    management system will select an appropriate parking spot.
    :parameter: Necessary JSON fields: 'id' (int), 'number_plate' (string),
        'length', 'width', 'turning_radius', 'dist_rear_axle_numberplate';
        Optional JSON fields:  'charge_type' (string), 'load' (boolean), 'near_exit' (boolean)
        and 'parking_card' (boolean).
    :return: HTTP response with status code 200 and JSON fields 'parking_in', which is set to true if the parking garage
        could register the vehicle, 'longitude' and 'latitude' for the target parking position and 'load_vehicle', which
        is set to true if the vehicle will be charged during parking.
    """
    if request.is_json:
        try:
            park_in_parameters = request.get_json()
            park_in_response = communication.communicate_park_in(park_in_parameters)
            if math.isnan(park_in_response["longitude"]) or math.isnan(park_in_response["latitude"]):
                return Response({'Vehicle status indicates an unsuccessful registration.'}, status=409)
            return jsonify({'parking_in': park_in_response["parking_in"],
                            'longitude': park_in_response["longitude"],
                            'latitude': park_in_response["latitude"],
                            'load_vehicle': park_in_response["load_vehicle"]})
        except communication.InternalCommunicationException as e:
            return Response({'Missing parameter in sent JSON: ' + str(e)}, status=422)
        except communication.CommunicationRosServiceException:
            return Response({communication_failed_message}, status=503)
    else:
        return Response({'Request had no JSON fields.'}, status=406)


@app.route('/parkin', methods=['POST'])
def perform_redirect_park_in():
    """
    Performs the park in process by calling perform_park_in_request.
    This route exists to avoid problems with CamelCase style of route parkIn.
    """
    return perform_park_in_request()


@app.route('/parkOut', methods=['POST'])
def perform_park_out_request():
    """
    This app route is called when an app user wants to park his vehicle out from the parking garage.
    The request must provide information about the vehicle to allow an identification.
    After the park out request, the parking management system will locate the vehicle and provide the coordinates of
    the zone where the car can be picked-up.
    :parameter: Necessary JSON fields: 'id' (int), 'number_plate' (string);
    :return: HTTP response with status code 200 and JSON fields 'parking_out', which is set to true if the parking
        garage could locate the vehicle, 'longitude' and 'latitude' representing the coordinates of the transfer zone.
    """
    if request.is_json:
        try:
            park_out_parameters = request.get_json()
            app_id = park_out_parameters["id"]
            number_plate = park_out_parameters["number_plate"]
            park_out_response = communication.communicate_park_out(app_id, number_plate)
            return jsonify({'parking_out': park_out_response["parking_out"],
                            'longitude': park_out_response["longitude"],
                            'latitude': park_out_response["latitude"]})
        except communication.InternalCommunicationException as e:
            return Response({'Missing parameter in sent JSON: ' + str(e)}, status=422)
        except communication.VehicleIdentificationException as e:
            return Response({str(e)}, status=406)
        except communication.CommunicationRosServiceException:
            return Response({communication_failed_message}, status=503)          
    else:
        return Response({'Request had no JSON fields.'}, status=406)


@app.route('/parkout', methods=['POST'])
def perform_redirect_park_out():
    """
    Performs the park out process by calling perform_park_out_request.
    This route exists to avoid problems with CamelCase style of route parkOut.
    """
    return perform_park_out_request()


@app.route('/getPosition', methods=['POST'])
def perform_get_position():
    """
    This route is called to get the current position of a vehicle, which can be shown on the garage animation.
    The request must provide information about the vehicle to allow an identification.
    After the get position request, the parking management system will search for the vehicle and return its current
    position as longitude and latitude.
    :parameter: Necessary JSON fields: 'id' and 'number_plate'
    :return: HTTP response with status code 200 and JSON fields 'longitude' and 'latitude', which provide the
        geographical coordinates of the vehicle´s position as well as boolean values for 'parking' and 'reached_position'
    """
    try:
        if request.is_json:
            get_position_parameters = request.get_json()
            app_id = get_position_parameters["id"]
            number_plate = get_position_parameters["number_plate"]
            position = communication.communicate_vehicle_position(app_id, number_plate)
            return jsonify({'longitude': position["longitude"],
                            'latitude': position["latitude"],
                            'parking': position["parking"],
                            'reached_position': position["reached_position"]})
    except communication.VehicleIdentificationException as e:
        return Response({str(e)}, status=406)
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/getposition', methods=['POST'])
def perform_redirect_get_position():
    """
    Gets current vehicle position by calling perform_get_position.
    This route exists to avoid problems with CamelCase style of route getPosition.
    """
    return perform_get_position


@app.route('/resetDatabase')
def perform_reset_database():
    """
    Route for operators to influence database content.
    It clears all content from the ID mapping database and initializes a new empty database.
    :return: Success message and status code 205 if database could be cleared
    """
    id_mapping.clear_db()
    id_mapping.init_db()
    return Response({"Database has been cleared. All IDs have been deleted. Vehicles must be registered again."},
                    status=205)


############################################################################
# Entry point for the program. Starting the application with url and port.
if __name__ == '__main__':
    id_mapping.init_db()
    app.run(debug=True, host=url_address, port=port, use_reloader=False)
