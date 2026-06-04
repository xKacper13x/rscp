import cobs
import cobs.cobs
import rscp_pb2
import io
import time
import serial


simulated_buffer = io.BytesIO(b'\x1c\x1a\x19\n\x17\tX9\xb4\xc8v\xbe\xf3?\x11\x83\xc0\xca\xa1E\xb6\x16@\x1d\xa1\xd6|?\x00')


def on_receive(data: bytes):
    # first step is to COBS decode the data
    cobs_decoded = cobs.cobs.decode(data)

    response = rscp_pb2.ResponseEnvelope()
    # next step is to decode(parse) the protobuf message
    response.ParseFromString(cobs_decoded)

    print(f"Received new message at: {time.time()}")
    print(f"Received Request type: {response.WhichOneof('response')}")
    print(response)


if __name__ == "__main__":
    buffer = b""
    while True:
        # instead of this, you may use serial.read() to read from serial port
        # see receive_commands.py for serial example
        data = simulated_buffer.read(1)
        if not data:
            break

        if data == b"\x00":
            buffer += b"\x00"
            break
            buffer = b""
        else:
            buffer += data

    ser = serial.Serial('/dev/pts/7', timeout=0.5,
                        baudrate=115200, parity='N', stopbits=1)
    ser.write(buffer)
    ser.flush()
    print('Sent data with x000')

    print('Listening...')
    buffer = b""
    while True:
        data = ser.read(1)

        if not data:
            continue

        if data == b"\x00":
            on_receive(buffer)
            buffer = b""
        else:
            buffer += data
