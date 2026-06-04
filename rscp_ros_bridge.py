from rscp_transceiver import RscpTransceiver
import time
import rscp_types


class RscpRosBridge:
    def __init__(self):
        self._transceiver = RscpTransceiver()
        self._transceiver.subscribe(self.receive_data)

    def receive_data(self, command: str, request):
        command_failed = False
        match command:
            case 'navigate_to_gps':
                # TODO: wysłać polecenia przez rosa
                pass

                # Tu będzie wysłanie komendy do ros
            case _:
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
        except TypeError:
            # Tu będzie wypisanie ostrzeżenia w rospy.logerr
            pass

    def send_battery_state(self, voltage: float, current: float,
                           state_of_charge: float):
        battery_state = rscp_types.BatteryState(voltage, current,
                                                state_of_charge)
        try:
            self._transceiver.send_message(battery_state)
        except TypeError:
            # Tu będzie wypisanie ostrzeżenia w rospy.logerr
            pass

    def send_distance(self, distance: float):
        measured_distance = rscp_types.MeasuredDistance(distance)
        try:
            self._transceiver.send_message(measured_distance)
        except TypeError:
            # Tu będzie wypisanie ostrzeżenia w rospy.logerr
            pass

    def send_rover_status(self):
        pass

    def send_task_finished(self):
        self._transceiver.send_task_finished()


if __name__ == '__main__':
    bridge = RscpRosBridge()
    while True:
        time.sleep(0.5)
