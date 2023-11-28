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
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Empty
from rclpy.duration import Duration
import rclpy
import math
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Defining Service 
from attach_shelf.srv import GoToLoading

# Shelf positions for picking
shelf_positions = {"loading_position": [4.11, 1.8143]} 

# Shipping destination for picked products
shipping_destinations = {"shipping_position": [-0.5249, 0.4031]}

# Initial position of the robot
initial_position = {"init_pos": [-0.418, 3.066]}

# Variables 
robot_status = False

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''
def create_circle(radius, num_points=20):
    circle = Polygon()
    angle_increment = 2 * math.pi / num_points

    for i in range(num_points):
        point = Point32()
        point.x = radius * math.cos(i * angle_increment)
        point.y = radius * math.sin(i * angle_increment)
        circle.points.append(point)

    return circle

def publish_shelf_unload_footprint(node):
    global_footprint_pub = node.create_publisher(Polygon, '/global_costmap/footprint', 10)
    local_footprint_pub = node.create_publisher(Polygon, '/local_costmap/footprint', 10)
    
    circle_msg = create_circle(radius=0.3)

    global_footprint_pub.publish(circle_msg)
    local_footprint_pub.publish(circle_msg)
    print("Changed Footprint to Unloaded Robot")
    time.sleep(1)

def publish_shelf_load_footprint(node):
    global_footprint_pub = node.create_publisher(Polygon,'/global_costmap/footprint', 10)
    local_footprint_pub = node.create_publisher(Polygon,'/local_costmap/footprint', 10)
    
    polygon_msg = Polygon()

    point1 = Point32()
    point1.x = 0.4
    point1.y = -0.4

    point2 = Point32()
    point2.x = 0.4
    point2.y = 0.4

    point3 = Point32()
    point3.x = -0.4
    point3.y = 0.4

    point4 = Point32()
    point4.x = -0.4
    point4.y = -0.4

    polygon_msg.points = [point1, point2, point3, point4]

    global_footprint_pub.publish(polygon_msg)
    local_footprint_pub.publish(polygon_msg)

def callback(msg):
    global robot_status
    robot_status = msg.data
    if msg.data:
        print("Shelf Ready to Move")
    else:
        print("Shelf not Ready to Move")

# Function for calling the service
def go_under_shelf():
    node = rclpy.create_node('path_manager')
    client = node.create_client(GoToLoading, 'approach_shelf')
    subscriber = node.create_subscription(Bool, 'shelf_ready', callback, 10)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Servicio no disponible, esperando...')

    solicitud = GoToLoading.Request()
    solicitud.attach_to_shelf = True  # Configura el valor según sea necesario

    future = client.call_async(solicitud)
    rclpy.spin_until_future_complete(node, future)

    while not robot_status:
        print("Waiting for the Shelf to be Ready")
        rclpy.spin_once(node, timeout_sec=1.0)

    if future.result() is not None:
        respuesta = future.result().complete
        node.get_logger().info('Shelf Loaded: %s' % respuesta)
        publish_shelf_load_footprint(node)
        print("FootPrint Changed")
    else:
        node.get_logger().warning('Error Calling the Service')

    node.destroy_node()

def unload_shelf():
    node = rclpy.create_node('unload_shelf')
    unload_shelf_pub = node.create_publisher(String,'/elevator_down', 20)
    empty_msg = String()
    empty_msg.data = ""
    unload_shelf_pub.publish(empty_msg)
    publish_shelf_unload_footprint(node)
    time.sleep(1)
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
    initial_pose.pose.position.x = -0.418
    initial_pose.pose.position.y = 3.066
    initial_pose.pose.orientation.z = -0.16
    initial_pose.pose.orientation.w = 0.98
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Sending the loading position coordinates
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = -0.8363  #-0.74
    shelf_item_pose.pose.orientation.w = 0.5481   #0.67
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
        navigator.clearAllCostmaps()

        print('Got shelf from ' + request_item_location +
              '! Bringing shelf to shipping destination (' + request_destination + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        shipping_destination.pose.orientation.z = 0.6104
        shipping_destination.pose.orientation.w = 0.7920
        navigator.goToPose(shipping_destination)
        
          
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
        unload_shelf()
        print('Left shelf in ' + request_destination +
              '! Going to Initial Position (' + init_position + ')...')
        navigator.goToPose(initial_pose)

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_destination +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_destination + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + init_position +
                  ' for robot: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Robot arrived to initial position.')

    elif result == TaskResult.CANCELED:
        print('Task at ' + init_position + ' was canceled. Returning to staging point...')
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
    print('Robot Ready to Operate')
    exit(0)


if __name__ == '__main__':
    main()
