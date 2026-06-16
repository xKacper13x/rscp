from scripts.rscp_transceiver import RscpTransceiver
import time
import scripts.rscp_types as rscp_types
import rospy
from sensor_msgs.msg import NavSatFix


class RscpRosBridge:
    def __init__(self):
        rospy.init_node("rscp_ros_bridge")
        self._timer = rospy.Timer(rospy.Duration(1.0), self._timer_callback)

        self._curr_rover_state = rscp_types.RoverState.DISARMED
        self._curr_heading = 0.0

        self._curr_battery_state = rscp_types.BatteryState(0.0, 0.0, 0.0)
        #self._battery_subscriber = rospy.Subscriber("/power_status", Nazwa_paczki,
        #                                             self._battery_callback)

        self._curr_gps_coordinate = rscp_types.GPSCoordinate(0.0, 0.0, 0.0)
        self._gps_subscriber = rospy.Subscriber("/gps/fix", NavSatFix,
                                                self._gps_callback)

        self._transceiver = RscpTransceiver()
        self._transceiver.subscribe(self.receive_data)

    def receive_data(self, command: str, request):
        command_failed = False
        if command == 'navigate_to_gps':
                # TODO: wysłać polecenia przez rosa
            pass
                # Tu będzie wysłanie komendy do ros
        elif command == 'arm_disarm':
            pass
        elif command == 'set_stage':
                # TODO: wysłać polecenia przez rosa
            pass
        elif command == 'search_area':
            # TODO: wysłać polecenia przez rosa
            pass
        elif command == 'set_stage':
                # TODO: wysłać polecenia przez rosa
            pass
        else:
            command_failed = True

        if not command_failed:
            self._transceiver.send_ack()

        # Kod tymczasowy do testowania
        print('Got command navigate_to_gps')
        latitude = request.navigate_to_gps.coordinate.latitude

        time.sleep(3)
        self.send_gps_coordinates(latitude, 4.20, 21.15)
        self.send_task_finished()

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
        # self._battery_data.voltage = data.
        self._curr_battery_state.current = data.battery1_current + data.battery2_current
        # self._battery_data.state_of_charge

    def _gps_callback(self, data):
        self._curr_gps_coordinate.latitude = data.latitude
        self._curr_gps_coordinate.longitude = data.longitude
        self._curr_gps_coordinate.altitude = data.altitude

if __name__ == '__main__':
    bridge = RscpRosBridge()
    print('Started RSCP ROS Bridge')

    rospy.spin()
