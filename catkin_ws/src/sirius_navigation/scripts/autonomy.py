#! /usr/bin/python3

import rospy
import struct
import threading
from std_msgs.msg import Float32, Empty, String
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from rscp_bridge.msg import AutonomyCommand
from geometry_msgs.msg import Twist
import pymap3d

Acknowledge = 0x00
ArmDisarm = 1
SetStage = 2
NavigateToGPS = 3
SearchArea = 4
StartExploration = 5

TaskCompleted = 0x03
Text = 0x05
ArucoTag = 0x06
LocateArucoTags = 0x07
Location3D = 0x08
Detection = 0x09
SetParameters = 0x0A

# map_origin_coords = (39.9013943, 32.7704792, 907.476)

# Testy pod marsjarderm
# latitude: 39.9013943
# longitude: 32.7704792
# altitude: 907.476

# Środek marsjardu
map_origin_coords = (39.9011333333333333, 32.77005, 907.476)

hold_repeater = True
lamp_color = "red"


def bits_to_float(value):
    value_bits = struct.pack("I", value)
    return struct.unpack("f", value_bits)[0]


class Message:

    def __init__(self, message_id, data):
        self.message_id = message_id
        self.data = data


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

        self.navigator = Navigator()
        self.stages_done = []

    def run(self):
        rospy.loginfo("Autonomy started")
        self.set_lamp("red")
        self.set_lamp("yellow")
        self.set_lamp("red")
        message = self.rs232.await_message(SetParameters)
        self.rs232.send_ack()

        while len(self.stages_done) < 3:
            message = self.rs232.await_message(SetStage)
            self.rs232.send_ack()
            self.stage = message["data"][0]
            rospy.loginfo(f"Satellite requested stage {self.stage}")
            if self.stage not in self.stages_done:
                if self.stage == 1:
                    self.stage_one()
                    self.stages_done.append(1)
                if self.stage == 2:
                    self.stage_two()
                    self.stages_done.append(2)
                if self.stage == 3:
                    self.stage_three()
                    self.stages_done.append(3)

        rospy.loginfo("Autonomy done. Thank you")

    def stage_one(self):
        rospy.loginfo("Beginning stage one")
        self.rs232.await_message(ArmDisarm)
        self.arm()
        self.set_lamp("green")
        self.rs232.send_ack()
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp("yellow")
        target = self.parse_navigation_command(message["data"])
        self.leave_airlock()
        self.navigate_to(*target)
        rospy.sleep(4)
        self.drop_repeater()
        self.set_lamp("green")
        self.rs232.send_task_completed()
        rospy.loginfo("Stage one done")

    def stage_two(self):
        rospy.loginfo("Beginning stage two")
        self.set_lamp("green")
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp("yellow")
        target = self.parse_navigation_command(message["data"])
        self.navigate_to(*target)
        self.set_lamp("green")
        self.rs232.send_task_completed()
        message = self.rs232.await_message(LocateArucoTags)
        self.rs232.send_ack()
        self.set_lamp("yellow")
        self.go_to_lavatube()
        self.rs232.send_task_completed()
        self.set_lamp("green")
        rospy.loginfo("Stage two done")

    def stage_three(self):
        rospy.loginfo("Beginning stage three")
        self.set_lamp("green")
        message = self.rs232.await_message(NavigateToGPS)
        self.rs232.send_ack()
        self.set_lamp("yellow")
        target = self.parse_navigation_command(message["data"])
        self.navigate_to(*target)
        self.set_lamp("green")
        self.rs232.send_task_completed()
        self.rs232.await_message(LocateArucoTags)
        self.rs232.send_ack()
        self.set_lamp("yellow")
        self.go_to_base()
        self.set_lamp("green")
        self.rs232.send_task_completed()
        self.rs232.await_message(ArmDisarm)
        self.rs232.send_ack()
        self.set_lamp("red")
        rospy.loginfo("Stage three done")

    def arm(self):
        rospy.loginfo("Rover armed")

    def set_lamp(self, color):
        global lamp_color
        rospy.loginfo(f"Setting the lamp to {color}")
        lamp_color = color

    def parse_navigation_command(self, command_data):
        lat_repr = (command_data[0] << 24
                    | command_data[1] << 16
                    | command_data[2] << 8
                    | command_data[3])
        long_repr = (command_data[4] << 24
                     | command_data[5] << 16
                     | command_data[6] << 8
                     | command_data[7])
        lat = bits_to_float(lat_repr)
        long = bits_to_float(long_repr)
        return (lat, long)

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
        pass

    def _autonomy_command_callback(self, data):
        command_id = data.command_id

        if command_id == 1:
            pass
        elif command_id == 2:
            self.stage = data.new_stage_num


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
