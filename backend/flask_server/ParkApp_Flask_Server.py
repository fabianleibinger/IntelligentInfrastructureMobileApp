import os
import configparser
from ctypes import c_uint

import rospy
import threading

from flask import Flask, jsonify, request, Response
from std_msgs.msg import String
from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse

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
threading.Thread(target=lambda: rospy.init_node(ros_root_node, disable_signals=True)).start()


def retrieve_free_parking_spots_from_pms():
    rospy.wait_for_service('capacity_request')
    try:
        capacity_request = rospy.ServiceProxy('capacity_request', CapacityRequest)
        capacities = capacity_request()
        return capacities
    except rospy.ServiceException as exception:
        print("Service call failed: %s" % exception)


def request_free_parking_spots(electric):
    current_capacities = retrieve_free_parking_spots_from_pms()
    if electric:
        free_electric = current_capacities.capacity_free.electric
        return jsonify({'request': 'capacity free electric', 'result': str(free_electric)})
    else:
        free_total = current_capacities.capacity_free.total
        return jsonify({'request': 'capacity free total', 'result': str(free_total)})


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


@app.route('/freeParkingSpots')
def get_normal_parking_spots():
    return request_free_parking_spots(False)


@app.route('/freeElectricParkingSpots')
def electric_parking_spots():
    return request_free_parking_spots(True)


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