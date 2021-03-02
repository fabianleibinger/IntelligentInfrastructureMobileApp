from flask import jsonify
import rospy
import threading

from std_msgs.msg import String
from parking_app_ros_pkg.srv import CapacityRequest, CapacityRequestResponse
from parking_app_ros_pkg.srv import RegisterVehicleRequest, RegisterVehicleRequestResponse


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


class CommunicationRosServiceException(Exception):
    pass


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
    

