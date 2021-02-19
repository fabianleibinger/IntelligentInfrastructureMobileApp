import os
import configparser
from enum import Enum

import parking_app_python_pkg.parking_communication as communication

from flask import Flask, request, Response

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
def connect():
    return


@app.route('/testJson')
def test_extract_content_from_json():
    print(request.is_json)
    content = request.get_json()
    print(content)
    return 'JSON posted'


@app.route('/capacities')
def get_capacities():
    return communication.request_capacities(False)


@app.route('/free')
def get_free_capacities():
    return communication.request_capacities(True)


@app.route('/free/total')
def all_free_parking_spots():
    return communication.request_free_parking_spots(False)


@app.route('/free/electric')
def electric_free_parking_spots():
    return communication.request_free_parking_spots(True)


@app.route('/parkIn')
def perform_park_in_request():
    return


@app.route('/parkOut')
def perform_park_out_request():
    return


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

