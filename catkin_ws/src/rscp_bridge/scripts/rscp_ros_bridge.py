from scripts.rscp_transceiver import RscpTransceiver
import scripts.rscp_types as rscp_types
import rospy
from sensor_msgs.msg import NavSatFix
# from canbus_modules.msg import PowerStatus
from rscp_bridge.msg import AutonomyCommand
from rscp_bridge.msg import AutonomyEvent


class RscpRosBridge:
    def __init__(self):
        rospy.init_node("rscp_ros_bridge")
        self._timer = rospy.Timer(rospy.Duration(1.0), self._timer_callback)

        self._curr_rover_state = rscp_types.RoverState.DISARMED
        self._curr_heading = 0.0

        self._curr_battery_state = rscp_types.BatteryState(0.0, 0.0, 0.0)
        # self._battery_subscriber = rospy.Subscriber("/power_status", PowerStatus,
        #                                             self._battery_callback)

        self._curr_gps_coordinate = rscp_types.GPSCoordinate(0.0, 0.0, 0.0)
        self._gps_subscriber = rospy.Subscriber("/gps/fix", NavSatFix,
                                                self._gps_callback)

        self._event_subscriber = rospy.Subscriber("/autonomy/events", AutonomyEvent,
                                                self._autonomy_event_callback)
        self._command_publisher = rospy.Publisher("/autonomy/commands", AutonomyCommand)

        self._transceiver = RscpTransceiver()
        self._transceiver.subscribe(self.receive_data)

    def receive_data(self, command: str, request):
        command_failed = False
        new_command = AutonomyCommand()

        if command == 'arm_disarm':
            new_command.command_id = 1
            new_command.arm_state = request.arm_disarm.value

        elif command == 'set_stage':
            new_command.command_id = 2
            new_command.new_stage_num = request.set_stage.value

        elif command == 'navigate_to_gps':
            latitude = request.navigate_to_gps.coordinate.latitude
            longitude = request.navigate_to_gps.coordinate.longitude

            new_command.command_id = 3
            new_command.latitude = latitude
            new_command.longitude = longitude

        elif command == 'search_area':
            radius = request.search_area.radius
            latitude = request.search_area.center_coordinate.latitude
            longitude = request.search_area.center_coordinate.longitude

            new_command.command_id = 4
            new_command.radius = radius
            new_command.latitude = latitude
            new_command.longitude = longitude

        elif command == 'start_exploration':
                new_command.command_id = 5
        else:
            command_failed = True

        if not command_failed:
            self._transceiver.send_ack()
            self._command_publisher.publish(new_command)

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
        self._battery_data.voltage = data.battery1_voltage
        self._curr_battery_state.current = data.battery1_current
        self._battery_data.state_of_charge = data.battery1_percentage

    def _gps_callback(self, data):
        self._curr_gps_coordinate.latitude = data.latitude
        self._curr_gps_coordinate.longitude = data.longitude
        self._curr_gps_coordinate.altitude = data.altitude

    def _autonomy_event_callback(self, data):
        if data.event_id == 0:
            self.send_task_finished()
        elif data.event_id == 1:
            self.send_gps_coordinates(data.latitude,
                                      data.longitude,
                                      data.altitude)
        elif data.event_id == 2:
            self.send_distance(data.distance)

if __name__ == '__main__':
    bridge = RscpRosBridge()

    rospy.spin()
