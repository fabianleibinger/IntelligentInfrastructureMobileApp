import os
import configparser
import rospy
import threading

from flask import Flask, jsonify, request
from std_msgs.msg import String


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
# Connect to ROS node
ros_root_node = 'parking_node'
ros_message = '/requestFreeParkingSpots01'
threading.Thread(target=lambda: rospy.init_node(ros_root_node, disable_signals=True)).start()
pubParkingSpots = rospy.Publisher(ros_root_node, String, queue_size=1)

############################################################################
# Implement logic here


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


@app.route('/freeParkingSpots', methods=['POST'])
def normal_parking_spots():
    return


@app.route('/freeElectricParkingSpots')
def electric_parking_spots():
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


############################################################################
# Entry point for the program. Starting the application with url and port.
if __name__ == '__main__':
    app.run(debug=True, host=url_address, port=port, use_reloader=False)
