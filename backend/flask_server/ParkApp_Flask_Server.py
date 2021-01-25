import os
import configparser
from flask import Flask, jsonify, request

#################################################################

app = Flask(__name__)  # server running on flask device

dir_path = os.path.dirname(os.path.realpath(__file__))  # file path of this file

config = configparser.RawConfigParser()
configFilePath = os.path.join(dir_path,
                              'config.properties')  # configFile should be saved in the same folder and named config.properties
config.read(configFilePath)

# Loading configuration from configuration file for url and port
url_address = config['server']['ipAddress']
port = config.getint('server', 'port')

##################################################################

# Implement logic here


##############################################################################

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


@app.route('/freeParkingSpots')
def get_free_parking_spots():
    return


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


#####################################################################

# Entry point for the program. Starting the application with url and port.
if __name__ == '__main__':
    app.run(debug=True, host=url_address, port=port, use_reloader=False)
