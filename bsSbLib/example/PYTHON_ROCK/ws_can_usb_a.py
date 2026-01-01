from enum import IntEnum
from pydantic import BaseModel
from typing import List
import serial
import time


DEBUG = False


def debug_print(*args):
    if DEBUG:
        print(*args)


class S_Protocol(IntEnum):
    FIXED_20_BYTE = 0x02
    VARIABLE = 0x12


class S_FrameType(IntEnum):
    STANDARD = 0x01
    EXTENDED = 0x02


class S_FrameFormat(IntEnum):
    DATA = 0
    REMOTE = 1


class S_CANMode(IntEnum):
    NORMAL = 0
    SILENT = 1
    LOOPBACK = 2
    LOOPBACK_SILENT = 3


class S_BaudRate(IntEnum):
    BR_1000K = 1
    BR_800K = 2
    BR_500K = 3
    BR_400K = 4
    BR_250K = 5
    BR_200K = 6
    BR_125K = 7
    BR_100K = 8
    BR_50K = 9
    BR_20K = 10
    BR_10K = 11
    BR_5K = 12


class CANSettings(BaseModel):
    protocol: S_Protocol
    frame_type: S_FrameType
    frame_format: S_FrameFormat
    can_mode: S_CANMode
    baudrate: S_BaudRate


class Waveshare_USBCAN_A:
    def __init__(self, serial_port: serial.Serial, can_settings: CANSettings):
        self.serial_port = serial_port
        self.settings = can_settings

        # Init driver
        self._init_driver()

    @staticmethod
    def calculate_checksum(data):
        checksum = sum(data[2:])
        return checksum & 0xFF

    def _init_driver(self):
        set_can_baudrate = [
            0xAA,  #  0  Packet header
            0x55,  #  1  Packet header
            self.settings.protocol.value,  #  3 Type: use variable protocol to send and receive data##  0x02- Setting (using fixed 20 byte protocol to send and receive data),   0x12- Setting (using variable protocol to send and receive data)##
            self.settings.baudrate.value,  #  3 CAN Baud Rate:  500kbps  ##  0x01(1Mbps),  0x02(800kbps),  0x03(500kbps),  0x04(400kbps),  0x05(250kbps),  0x06(200kbps),  0x07(125kbps),  0x08(100kbps),  0x09(50kbps),  0x0a(20kbps),  0x0b(10kbps),   0x0c(5kbps)##
            self.settings.frame_type.value,  #  4  Frame Type: Extended Frame  ##   0x01 standard frame,   0x02 extended frame ##
            0x00,  #  5  Filter ID1
            0x00,  #  6  Filter ID2
            0x00,  #  7  Filter ID3
            0x00,  #  8  Filter ID4
            0x00,  #  9  Mask ID1
            0x00,  #  10 Mask ID2
            0x00,  #  11 Mask ID3
            0x00,  #  12 Mask ID4
            self.settings.can_mode.value,  #  13 CAN mode:  normal mode  ##   0x00 normal mode,   0x01 silent mode,   0x02 loopback mode,   0x03 loopback silent mode ##
            0x00,  #  14 automatic resend:  automatic retransmission
            0x00,  #  15 Spare
            0x00,  #  16 Spare
            0x00,  #  17 Spare
            0x00,  #  18 Spare
        ]

        # Calculate checksum
        checksum = Waveshare_USBCAN_A.calculate_checksum(set_can_baudrate)
        set_can_baudrate.append(checksum)
        debug_print(set_can_baudrate)
        set_can_baudrate = bytes(set_can_baudrate)

        # Send command to set CAN baud rate

        num_set_baud = self.serial_port.write(set_can_baudrate)
        debug_print(f"Set CAN baud rate command sent, bytes written: {num_set_baud}")

        time.sleep(1)

    def _construct_can_frame(self, identifier: int, message: int, data_len: int):
        frame = []

        # ------ Header ------
        frame.append(0xAA)

        type = 0
        type |= data_len
        type |= (self.settings.frame_format == S_FrameFormat.REMOTE) << 4
        type |= (self.settings.frame_type == S_FrameType.EXTENDED) << 5
        type |= 1 << 6  # Reserved
        type |= 1 << 7  # Reserved

        frame.append(type)

        # ------ Identifier ------
        if self.settings.frame_format == S_FrameType.EXTENDED:
            id29 = identifier & 0x7FFFFFF
            frame.append((id29) & 0xFF)
            frame.append(((id29 >> 8) & 0xFF))
            frame.append(((id29 >> 16) & 0xFF))
            frame.append(((id29 >> 24) & 0xFF))
        else:
            id11 = identifier & 0x7FF
            frame.append((id11) & 0xFF)
            frame.append((id11 >> 8) & 0xFF)

        # ------ Data Field ------
        frame.append((message >> 0) & 0xFF)
        frame.append((message >> 8) & 0xFF)
        frame.append((message >> 16) & 0xFF)
        frame.append((message >> 24) & 0xFF)

        if self.settings.frame_format == S_FrameType.EXTENDED:
            frame.append((message >> 32) & 0xFF)
            frame.append((message >> 40) & 0xFF)
            frame.append((message >> 48) & 0xFF)
            frame.append((message >> 56) & 0xFF)

        # ------ End byte ------
        frame.append(0x55)

        return frame

    def transmit(self, identifier: int, data: int, data_len: int):
        can_frame = bytes(self._construct_can_frame(identifier, data, data_len))

        debug_print(f"(Transmit) {can_frame}")

        return self.serial_port.write(can_frame) is not None

    def receive(self, buffer: List, buffer_size: int, timeout_ms: float):
        time_stamp = time.monotonic()
        start_byte_detected = False
        msg_type_byte_discarded = False
        MAX_DATA_BUFFER = 10 if (self.settings.frame_type == S_FrameType.EXTENDED) else 6

        data_buffer = []
        debug_print(f"(Receive) Buffer size -> {buffer_size}")

        while True:
            serial_buffer = self.serial_port.read_all()

            if serial_buffer:
                debug_print(f"(Receive) {' '.join(f'{b:02X}' for b in serial_buffer)}")

                time_stamp = time.monotonic()
                for byte in serial_buffer:
                    # start byte
                    if byte == 0xAA and not start_byte_detected:
                        start_byte_detected = True
                        msg_type_byte_discarded = False
                        continue

                    # end byte
                    if byte == 0x55 and len(data_buffer) == MAX_DATA_BUFFER:
                        # Write into output
                        data = 0

                        if self.settings.frame_type == S_FrameType.EXTENDED:
                            data |= (data_buffer[9]) << 56
                            data |= (data_buffer[8]) << 48
                            data |= (data_buffer[7]) << 40
                            data |= (data_buffer[6]) << 32

                        data |= (data_buffer[5]) << 24
                        data |= (data_buffer[4]) << 16
                        data |= (data_buffer[3]) << 8
                        data |= data_buffer[2]

                        debug_print(f"(Receive) data-> {hex(data)}")
                        buffer.append(data)

                        data_buffer.clear()
                        start_byte_detected = False
                        msg_type_byte_discarded = True
                        continue

                    # Rogue Bytes
                    if not start_byte_detected:
                        continue

                    # Message Type
                    if not msg_type_byte_discarded:
                        msg_type_byte_discarded = True
                        continue

                    data_buffer.append(int(byte))

            if len(buffer) >= buffer_size:
                break

            end = time.monotonic()
            duration = end - time_stamp

            if duration > (timeout_ms / 1000):
                break

        return len(buffer)
