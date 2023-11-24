#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Defining Service 
from attach_shelf.srv import GoToLoading

# Shelf positions for picking
shelf_positions = {"loading_position": [5.80, -0.55]} #5.78

# Shipping destination for picked products
shipping_destinations = {"shipping_position": [0.324, -3.021]}

# Initial position of the robot
initial_position = {"init_pos": [0.007, -0.005]}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''
# Function for calling the service
def go_under_shelf():
    node = rclpy.create_node('path_manager')
    client = node.create_client(GoToLoading, 'approach_shelf')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Servicio no disponible, esperando...')

    solicitud = GoToLoading.Request()
    solicitud.attach_to_shelf = True  # Configura el valor según sea necesario

    future = client.call_async(solicitud)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        respuesta = future.result().complete
        node.get_logger().info('Resultado de la llamada al servicio: %s' % respuesta)
    else:
        node.get_logger().warning('Error al llamar al servicio')

    node.destroy_node()

def main():

    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    init_position = 'init_pos'
    request_item_location = 'loading_position'
    request_destination = 'shipping_position'
    ####################

    rclpy.init()

    # Defining Service 
    # node = rclpy.create_node('path_manager')
    # client = node.create_client(GoToLoading, 'approach_shelf')

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.007
    initial_pose.pose.position.y = -0.005
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = -0.021
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Sending the loading position coordinates
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = -0.90  #-0.74
    shelf_item_pose.pose.orientation.w = 0.9   #0.67
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for shelf: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    # Read the result of the previous goal
    result = navigator.getResult()

    # Sending the shipping position coordinates
    if result == TaskResult.SUCCEEDED:
        # If robot get in the loading position then call the service to load the shelf 
        print ('Attemping to load the shelf')
        go_under_shelf()
        '''
        print('Got shelf from ' + request_item_location +
              '! Bringing shelf to shipping destination (' + request_destination + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = 1.59
        navigator.goToPose(shipping_destination)
        '''
    '''        
    elif result == TaskResult.CANCELED:
        print('Task at ' + request_destination +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_destination + ' failed!')
        exit(-1)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_destination +
                  ' for shelf: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    # Sending the initial position coordinates
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Left shelf in ' + request_destination +
              '! Going to Initial Position (' + init_position + ')...')
        navigator.goToPose(initial_pose)

    elif result == TaskResult.CANCELED:
        print('Task at ' + init_position +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + init_position + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + init_position +
                  ' for robot: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
    '''
    exit(0)


if __name__ == '__main__':
    main()
