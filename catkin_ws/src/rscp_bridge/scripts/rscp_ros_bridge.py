#!/usr/bin/env python3
from rscp_transceiver import RscpTransceiver
import rscp_types
import rospy
from sensor_msgs.msg import NavSatFix
# from canbus_modules.msg import PowerStatus
from rscp_bridge.msg import AutonomyCommand
from rscp_bridge.msg import AutonomyEvent
import enum


class CommandType(enum.IntEnum):
    ARM_DISARM = 1
    SET_STAGE = 2
    NAVIGATE_TO_GPS = 3
    SEARCH_AREA = 4
    START_EXPLORATION = 5


class EventType(enum.IntEnum):
    TASK_FINISHED = 0
    LOCATION = 1
    DISTANCE = 2


class RscpRosBridge:
    def __init__(self):
        rospy.init_node('rscp_ros_bridge')

        self._curr_rover_state = rscp_types.RoverState.DISARMED
        self._curr_heading = 0.0
        self._curr_battery_state = rscp_types.BatteryState(0.0, 0.0, 0.0)
        self._curr_gps_coordinate = rscp_types.GPSCoordinate(0.0, 0.0, 0.0)

        self._command_publisher = rospy.Publisher("/autonomy/commands",
                                                  AutonomyCommand, queue_size=10)

        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)

        self._transceiver = RscpTransceiver(port, baudrate)
        self._transceiver.subscribe(self.receive_data)

        self._timer = rospy.Timer(rospy.Duration(1.0), self._timer_callback)

        # self._battery_subscriber = rospy.Subscriber("/power_status", PowerStatus,
        #                                             self._battery_callback)

        self._gps_subscriber = rospy.Subscriber("/gps/fix", NavSatFix,
                                                self._gps_callback)

        self._event_subscriber = rospy.Subscriber("/autonomy/events", AutonomyEvent,
                                                  self._autonomy_event_callback)

    def receive_data(self, command: str, request):
        command_failed = False
        new_command = AutonomyCommand()

        if command == 'arm_disarm':
            new_command.command_id = CommandType.ARM_DISARM
            new_command.arm_state = request.arm_disarm.value

        elif command == 'set_stage':
            new_command.command_id = CommandType.SET_STAGE
            new_command.new_stage_num = request.set_stage.value

        elif command == 'navigate_to_gps':
            latitude = request.navigate_to_gps.coordinate.latitude
            longitude = request.navigate_to_gps.coordinate.longitude

            new_command.command_id = CommandType.NAVIGATE_TO_GPS
            new_command.latitude = latitude
            new_command.longitude = longitude

        elif command == 'search_area':
            radius = request.search_area.radius
            latitude = request.search_area.center_coordinate.latitude
            longitude = request.search_area.center_coordinate.longitude

            new_command.command_id = CommandType.SEARCH_AREA
            new_command.radius = radius
            new_command.latitude = latitude
            new_command.longitude = longitude

        elif command == 'start_exploration':
            new_command.command_id = CommandType.START_EXPLORATION
        else:
            command_failed = True

        if not command_failed:
            self._transceiver.send_ack()
            self._command_publisher.publish(new_command)
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def send_gps_coordinates(self, latitude: float,
                             longitude: float, altitude: float):
        gps_coordinates = rscp_types.GPSCoordinate(latitude,
                                                   longitude,
                                                   altitude)
        try:
            self._transceiver.send_message(gps_coordinates)
        except TypeError as e:
            rospy.logerr(e)

    def send_battery_state(self, voltage: float, current: float,
                           state_of_charge: float):
        battery_state = rscp_types.BatteryState(voltage, current,
                                                state_of_charge)
        try:
            self._transceiver.send_message(battery_state)
        except TypeError as e:
            rospy.logerr(e)

    def send_distance(self, distance: float):
        measured_distance = rscp_types.MeasuredDistance(distance)
        try:
            self._transceiver.send_message(measured_distance)
        except TypeError as e:
            rospy.logerr(e)

    def send_task_finished(self):
        self._transceiver.send_task_finished()

    def _timer_callback(self, event):
        curr_status = rscp_types.RoverStatus(self._curr_rover_state,
                                             self._curr_gps_coordinate,
                                             self._curr_heading,
                                             self._curr_battery_state)
        self._transceiver.send_message(curr_status)

    def _battery_callback(self, data):
        self._curr_battery_state.voltage = data.battery1_voltage
        self._curr_battery_state.current = data.battery1_current
        self._curr_battery_state.state_of_charge = data.battery1_percentage

    def _gps_callback(self, data):
        self._curr_gps_coordinate.latitude = data.latitude
        self._curr_gps_coordinate.longitude = data.longitude
        self._curr_gps_coordinate.altitude = data.altitude

    def _autonomy_event_callback(self, data):
        if data.event_id == EventType.TASK_FINISHED:
            self.send_task_finished()
        elif data.event_id == EventType.LOCATION:
            self.send_gps_coordinates(data.latitude,
                                      data.longitude,
                                      data.altitude)
        elif data.event_id == EventType.DISTANCE:
            self.send_distance(data.distance)

if __name__ == '__main__':
    bridge = RscpRosBridge()

    rospy.spin()
