from typing import List
from nav2_msgs.action import FollowWaypoints
from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin import State, StateMachine
from yasmin_ros import ActionState
from geometry_msgs.msg import PoseStamped

from threading import Lock, Condition
import random

from tb3_yasmin_msgs.srv import Mode


class RandomState(State):
    def __init__(self, waypoints, blackboard_lock, client) -> None:
        super().__init__(outcomes=[Mode.Request.SEQUENTIAL, Mode.Request.IDLE])
        self.waypoints = waypoints
        self.blackboard_lock = blackboard_lock
        self.client = client

    def execute(self, blackboard: Blackboard) -> str:
        finish = False
        goal = FollowWaypoints.Goal()
        pose = PoseStamped()

        while not finish:
            random_waypoint = random.choice(self.waypoints)
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = random_waypoint[0]
            pose.pose.position.y = random_waypoint[1]

            goal.poses.append(pose)

            self.client.wait_for_server()
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.is_succeeded():
                print("Waypoint reached")

            elif self.client.is_canceled():
                self.blackboard_lock.acquire()
                mode = blackboard.mode
                self.blackboard_lock.release()

                return mode


class SequentialState(State):
    def __init__(self, waypoints, blackboard_lock, client) -> None:
        super().__init__(outcomes=[Mode.Request.RANDOM, Mode.Request.IDLE])
        self.client = client
        self.waypoints = waypoints
        self.blackboard_lock = blackboard_lock

    def execute(self, blackboard: Blackboard) -> str:
        finish = False
        latest_waypoint = 0
        goal = FollowWaypoints.Goal()
        pose = PoseStamped()

        while not finish:
            next_waypoint = self.waypoints[latest_waypoint]
            latest_waypoint = (latest_waypoint + 1) % len(self.waypoints)
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = next_waypoint[0]
            pose.pose.position.y = next_waypoint[1]

            goal.poses.append(pose)

            self.client.wait_for_server()
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.is_succeeded():
                print("Waypoint reached")

            elif self.client.is_canceled():
                self.blackboard_lock.acquire()
                mode = blackboard.mode
                self.blackboard_lock.release()

                return mode


class WaypointNavigation(Node):
    def __init__(self):
        super().__init__("waypoint_navigation")

        self.blackboard = Blackboard()
        self.blackboard_lock = Lock()
        self.idle_condition = Condition()

        self.create_service(Mode, "mode", self.mode_callback)
        self.client = self.create_action_client(FollowWaypoints, "follow_waypoints")

    def mode_callback(self, request: Mode.Request, response: Mode.Response):
        if request.mode not in [
            Mode.Request.RANDOM,
            Mode.Request.SEQUENTIAL,
            Mode.Request.IDLE,
        ]:
            response.result = f"Invalid mode {request.mode}"
            return response

        curr_mode = request.mode
        self.blackboard_lock.acquire()
        if curr_mode != self.blackboard.mode:
            self.blackboard.mode = curr_mode

            if curr_mode != Mode.Request.IDLE:
                self.client.cancel_goal()
            else:
                self.idle_condition.notify()

        self.blackboard_lock.release()

        response.result = "OK"
        return response
