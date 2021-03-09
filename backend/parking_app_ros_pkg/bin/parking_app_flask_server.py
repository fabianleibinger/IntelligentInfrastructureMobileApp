import os
import configparser
from flask import Flask, jsonify, request, Response, redirect

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

# Loading configuration from configuration file for url and port
url_address = config['server']['ipAddress']
port = config.getint('server', 'port')

parking_garage_name = config['parking garage']['name']

############################################################################

communication_failed_message = "The parking garage management system could not return the current capacity."


############################################################################
# Routes
@app.route('/')
def main():
    """
    Main http route. Welcomes user on server.
    :return: Welcome message
    """
    return "Welcome to the server. "


@app.route('/connect')
def return_connection_information():
    return jsonify({'IP': url_address, 'Port': port, 'Parking garage': parking_garage_name})


@app.route('/testJson')
def test_extract_content_from_json():
    print(request.is_json)
    content = request.get_json()
    print(content)
    return 'JSON posted'


@app.route('/capacities', methods=['GET', 'POST'])
def get_capacities():
    try:
        return communication.request_capacities(False)
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free', methods=['GET', 'POST'])
def get_free_capacities():
    try:
        return communication.request_capacities(True)
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free/total', methods=['GET', 'POST'])
def all_free_parking_spots():
    try:
        return communication.request_free_parking_spots(False)
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/free/electric', methods=['GET', 'POST'])
def electric_free_parking_spots():
    try:
        return communication.request_free_parking_spots(True)
    except communication.CommunicationRosServiceException:
        return Response({communication_failed_message}, status=503)


@app.route('/parkIn', methods=['POST'])
def perform_park_in_request():
    if request.is_json:
        try:
            park_in_parameters = request.get_json()
            return communication.communicate_park_in(park_in_parameters)
        except communication.InternalCommunicationException as e:
            return Response({'Missing parameter in sent JSON: ' + str(e)}, status=422)
    else:
        return Response({'Request had no JSON fields.'}, status=406)


@app.route('/parkin', methods=['POST'])
def perform_redirect_park_in():
    return perform_park_in_request()


@app.route('/parkOut', methods=['POST'])
def perform_park_out_request():
    return


@app.route('/parkout', methods=['POST'])
def perform_redirect_park_out():
    return perform_park_out_request()


@app.route('/getPosition')
def get_position():
    return


@app.route('/garageAnimation')
def show_garage_animation():
    return


############################################################################
# Entry point for the program. Starting the application with url and port.
if __name__ == '__main__':
    app.run(debug=True, host=url_address, port=port, use_reloader=False)


