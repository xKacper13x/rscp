#! /usr/bin/python3

import rospy
import threading
from dataclasses import dataclass
import queue
from std_msgs.msg import Float32, Empty, String
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from rscp_bridge.msg import AutonomyCommand, AutonomyEvent
from geometry_msgs.msg import Twist
import pymap3d

ArmDisarm = 1
SetStage = 2
NavigateToGPS = 3
SearchArea = 4
StartExploration = 5

TaskCompleted = 0
Location3D = 1
Distance = 2

# map_origin_coords = (39.9013943, 32.7704792, 907.476)

# Testy pod marsjarderm
# latitude: 39.9013943
# longitude: 32.7704792
# altitude: 907.476

# Środek marsjardu
map_origin_coords = (39.9011333333333333, 32.77005, 907.476)

hold_repeater = True
lamp_color = "red"


class Message:

    def __init__(self, message_id, data):
        self.message_id = message_id
        self.data = data


@dataclass
class EventMessage:
    event_id: int

    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    length: float = 0.0


class Navigator:

    def __init__(self):
        self.move_base_action = SimpleActionClient("move_base/move_base",
                                                   MoveBaseAction)
        #self.move_base_action.wait_for_server()
        self.state = "ok"

    def start_navigating_to(self, pose: PoseStamped):
        self.state = "ok"
        rospy.loginfo("Setting goal")
        self.move_base_action.send_goal(MoveBaseGoal(target_pose=pose),
                                        self.move_base_done)
        rospy.loginfo("Goal sent")

    def move_base_done(self, terminal_state, result):
        self.state = "done"

    def get_status(self):
        failed_status = {
            GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED,
            GoalStatus.RECALLED, GoalStatus.LOST
        }
        if self.move_base_action.get_state() in failed_status:
            self.state = "error"
        return self.state

    def stop_navigating(self):
        self.move_base_action.cancel_all_goals()


class Autonomy:

    def __init__(self):
        rospy.init_node("autonomy")

        self._command_subscriber = rospy.Subscriber("/autonomy/commands", AutonomyCommand,
                                                    self._autonomy_command_callback)
        self._event_publisher = rospy.Publisher("/autonomy/events", AutonomyEvent)

        self.navigator = Navigator()
        self._command_queue = queue.Queue()
        self.stages_done = []

    def run(self):
        rospy.loginfo("Autonomy started")
        self.set_lamp("red")
        self.set_lamp("yellow")
        self.set_lamp("red")

        while len(self.stages_done) < 4:
            message = self.await_message(SetStage)
            stage = message.new_stage_num
            rospy.loginfo(f"Satellite requested stage {stage}")
            if stage not in self.stages_done:
                if stage == 1:
                    self.stage_one()
                    self.stages_done.append(1)
                if stage == 2:
                    self.stage_two()
                    self.stages_done.append(2)
                if stage == 3:
                    self.stage_three()
                    self.stages_done.append(3)
                if stage == 4:
                    self.stage_four()
                    self.stages_done.append(4)

        rospy.loginfo("Autonomy done. Thank you")

    def stage_one(self):
        rospy.loginfo("Beginning stage one")
        self.set_lamp("red")
        self.await_message(ArmDisarm)
        self.arm()
        self.set_lamp("green")
        message = self.await_message(SearchArea)
        self.set_lamp("yellow")

        self.leave_airlock()
        self.navigate_to(message.latitude, message.longitude)
        rospy.sleep(4)
        self.drop_repeater()

        # Otrzymanie wiadomości o ukończeniu zadania
        # TODO: Dodać prawdziwy GPS
        event_message = EventMessage(Location3D)
        self.send_event_message(event_message)
        self.set_lamp("green")

        event_message.event_id = TaskCompleted
        self.send_event_message(event_message)
        rospy.loginfo("Stage one done")

    def stage_two(self):
        rospy.loginfo("Beginning stage two")
        self.set_lamp("green")

        message = self.await_message(SearchArea)
        self.set_lamp("yellow")
        self.navigate_to(message.latitude, message.longitude)

        # Znaleziono kamień
        # TODO: Dodać prawdziwy GPS
        event_message = EventMessage(Location3D)
        self.send_event_message(event_message)
        self.set_lamp("green")

        event_message.event_id = TaskCompleted
        self.send_event_message(event_message)
        rospy.loginfo("Stage two done")

    def stage_three(self):
        rospy.loginfo("Beginning stage three")
        self.set_lamp("green")
        message = self.await_message(NavigateToGPS)

        self.set_lamp("yellow")
        self.navigate_to(message.latitude, message.longitude)

        self.set_lamp("green")
        event_message = EventMessage(TaskCompleted)
        self.send_event_message(event_message)
        # self.locate_tube_entrance()
        self.await_message(StartExploration)
        self.set_lamp("yellow")

        # Tu jest mierzenie tunelu
        tube_length = 0
        event_message = EventMessage(Distance,
                                     length=tube_length)
        self.send_event_message(event_message)

        event_message.event_id = TaskCompleted
        self.send_event_message(event_message)
        self.set_lamp("green")
        rospy.loginfo("Stage three done")

    def stage_four(self):
        rospy.loginfo("Beginning stage four")
        self.set_lamp("green")
        message = self.await_message(NavigateToGPS)

        self.set_lamp("yellow")
        self.navigate_to(message.latitude, message.longitude)

        self.set_lamp("green")
        event_message = EventMessage(TaskCompleted)
        self.send_event_message(event_message)

        self.set_lamp("yellow")
        # self.locate_airlock_entrance()
        self.await_message(ArmDisarm)
        self.set_lamp("red")
        rospy.loginfo("Mission completed")

    def arm(self):
        rospy.loginfo("Rover armed")

    def set_lamp(self, color):
        global lamp_color
        rospy.loginfo(f"Setting the lamp to {color}")
        lamp_color = color

    def leave_airlock(self):
        cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        deadline = rospy.Time.now() + rospy.Duration(40)
        command = Twist()
        command.linear.x = 0.1
        command.angular.z = -0.005
        rate = rospy.Rate(10)
        while (rospy.Time.now() < deadline and not rospy.is_shutdown()):
            cmd_vel_publisher.publish(command)
            rate.sleep()

    def navigate_to(self, lat, long):
        rospy.loginfo(f"Navigating to coords: {lat} {long}")
        deadline = rospy.Time.now() + rospy.Duration(8 * 60)
        target = pymap3d.geodetic2enu(lat, long, map_origin_coords[2],
                                      map_origin_coords[0],
                                      map_origin_coords[1],
                                      map_origin_coords[2])
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.position.z = target[2]
        pose.pose.orientation.w = 1
        self.navigator.start_navigating_to(pose)
        status = "ok"
        while status != "done" and not rospy.is_shutdown():
            status = self.navigator.get_status()
            if status == "error":
                self.navigator.start_navigating_to(pose)
            if rospy.Time.now() > deadline:
                self.navigator.stop_navigating()
                rospy.loginfo("Navigation timeout. Terminating execution")
                return
        rospy.loginfo("Target reached")

    def drop_repeater(self):
        global hold_repeater
        hold_repeater = False
        rospy.loginfo(f"Dropping the repeater")

    def go_to_lavatube(self):
        rospy.loginfo(f"Go to lavatube entrance")
        # self.navigate_to(*lavatube_coords)

    def go_to_base(self):
        rospy.loginfo(f"Returning to base")
        # self.navigate_to(*base_coords)

    def await_message(self, expected_command_id):
        while not rospy.is_shutdown():
            next_command = self._command_queue.get()
            command_id = next_command.command_id
            if command_id == expected_command_id:
                return next_command

    def send_event_message(self, data):
        message = AutonomyEvent()

        message.event_id = data.event_id
        if data.event_id == Location3D:
            message.latitude = data.latitude
            message.longitude = data.longitude
            message.altitude = data.altitude
        elif data.event_id == Distance:
            message.distance = data.length

        self._event_publisher.publish(message)

    def _autonomy_command_callback(self, data):
        self._command_queue.put(data)


if __name__ == "__main__":
    autonomy = Autonomy()
    rate = rospy.Rate(1)
    force_publisher = rospy.Publisher("/gripper/set_force",
                                      Float32,
                                      queue_size=10)
    open_publisher = rospy.Publisher("/gripper/open_trigger",
                                     Empty,
                                     queue_size=10)
    lamp_override_publisher = rospy.Publisher("lamps/color_override",
                                              String,
                                              queue_size=10)

    def publisher_worker():
        has_opened = False
        opening_duration = rospy.Duration(10)
        opening_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if has_opened and rospy.Time.now(
            ) > opening_start_time + opening_duration:
                break
            if hold_repeater:
                force_publisher.publish(Float32(1))
            else:
                open_publisher.publish(Empty())
                if not has_opened:
                    has_opened = True
                    opening_start_time = rospy.Time.now()
            rate.sleep()

    def lamp_publisher_worker():
        while not rospy.is_shutdown():
            lamp_override_publisher.publish(String(lamp_color))
            rate.sleep()

    publisher_thread = threading.Thread(target=publisher_worker)
    publisher_thread.start()
    lamp_publisher_thread = threading.Thread(target=lamp_publisher_worker)
    lamp_publisher_thread.start()
    autonomy.run()
