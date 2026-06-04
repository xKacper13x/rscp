import cobs
import cobs.cobs
import rscp_pb2
import serial
import threading
import google.protobuf
import rscp_types


class RscpTransceiver:
    def __init__(self):
        self._serial = serial.Serial('/dev/pts/6', timeout=0.5,
                                     baudrate=115200, parity='N', stopbits=1)
        self._callback_function = None

        reading_thread = threading.Thread(target=self._listen_serial,
                                          daemon=True)
        reading_thread.start()

    def subscribe(self, callback_func) -> None:
        self._callback_function = callback_func

    def send_ack(self):
        response = rscp_pb2.ResponseEnvelope()
        response.acknowledge.SetInParent()
        self.send_data(response)

    def send_task_finished(self):
        response = rscp_pb2.ResponseEnvelope()
        response.task_finished.SetInParent()
        self.send_data(response)

    def send_message(self, payload):
        response = rscp_pb2.ResponseEnvelope()

        match payload:
            case rscp_types.GPSCoordinate():
                response.gps_coordinate.SetInParent()
                response.gps_coordinate.latitude = payload.latitude
                response.gps_coordinate.longitude = payload.longitude
                response.gps_coordinate.altitude = payload.altitude

            case rscp_types.RoverStatus():
                response.rover_status.state = payload.state

                response.rover_status.coordinate.latitude = payload.coordinate.latitude
                response.rover_status.coordinate.longitude = payload.coordinate.longitude
                response.rover_status.coordinate.altitude = payload.coordinate.altitude

                response.rover_status.heading = payload.heading

                response.rover_status.battery_state.voltage = payload.battery_state.voltage
                response.rover_status.battery_state.current = payload.battery_state.current
                response.rover_status.battery_state.state_of_charge = payload.battery_state.state_of_charge

            case rscp_types.MeasuredDistance():
                response.distance = payload.distance

            case _:
                raise TypeError("Given type does not match any of available messages.")

        self.send_data(response)

    def send_data(self, response):
        serialized_bytes = response.SerializeToString()
        cobs_coded = cobs.cobs.encode(serialized_bytes)
        cobs_coded += b"\x00"

        self._serial.write(cobs_coded)

    def _on_receive(self, raw_data: bytes):
        cobs_decoded = cobs.cobs.decode(raw_data)
        request = rscp_pb2.RequestEnvelope()
        request.ParseFromString(cobs_decoded)

        command = request.WhichOneof('request')
        if self._callback_function is not None:
            self._callback_function(command, request)

    def _listen_serial(self):
        buffer = b""
        while True:
            new_byte = self._serial.read(1)
            if not new_byte:
                continue
            if new_byte == b"\x00":
                try:
                    self._on_receive(buffer)
                except cobs.cobs.DecodeError as e:
                    print(f'DecodeError: {e}')
                except google.protobuf.message.DecodeError as e:
                    print(f'Protobuf DecodeError: {e}')

                buffer = b""
            else:
                buffer += new_byte
                if len(buffer) > 1024:
                    buffer = b""
