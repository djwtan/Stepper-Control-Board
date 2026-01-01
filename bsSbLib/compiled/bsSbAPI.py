import ctypes
import os
import asyncio
import enum
from typing import Callable, Type, TypeVar
from pydantic import BaseModel, Field

DEBUG = True


def debug_print(*args):
    if DEBUG:
        print(*args)


# ==================================================================================== #
#                                      Export .so                                      #
# ==================================================================================== #

PATH = os.path.dirname(os.path.abspath(__file__)) + "/libbsSb.so"

BSSBLIB = ctypes.CDLL(PATH)


# Queue status struct
class _QueueStatus(ctypes.Structure):
    _fields_ = [
        ("is_executing", ctypes.c_uint16),
        ("motion_num", ctypes.c_uint16),
        ("sequence_num", ctypes.c_uint16),
    ]


# Board Info / SD Card
BSSBLIB.readFirmwareVersion.restype = ctypes.c_bool
BSSBLIB.readDriverConfig.restype = ctypes.c_bool
BSSBLIB.readMotionConfig.restype = ctypes.c_bool
BSSBLIB.readSequenceConfig.restype = ctypes.c_bool
BSSBLIB.readBuffer.restype = ctypes.c_bool
BSSBLIB.readPath.restype = ctypes.c_bool
BSSBLIB.readFileAtPath.restype = ctypes.c_bool
BSSBLIB.deleteFileAtPath.restype = ctypes.c_bool
BSSBLIB.writeToFileAtPath.restype = ctypes.c_bool

# Driver Readback
BSSBLIB.clearQueuedReadback.restype = None
BSSBLIB.getQueueReadback.restype = ctypes.c_bool

# Move Commands
BSSBLIB.q_movePreloaded.restype = ctypes.c_bool
BSSBLIB.q_moveStepper.restype = ctypes.c_bool
BSSBLIB.q_moveStepper_homing.restype = ctypes.c_bool
BSSBLIB.q_moveStepper_inverseTime.restype = ctypes.c_bool
BSSBLIB.q_moveStepper_vibration.restype = ctypes.c_bool


# Control Word
BSSBLIB.rampStop.restype = ctypes.c_bool
BSSBLIB.emergencyStop.restype = ctypes.c_bool
BSSBLIB.resetPosition.restype = ctypes.c_bool
BSSBLIB.enableDriver.restype = ctypes.c_bool
BSSBLIB.releaseDriver.restype = ctypes.c_bool

# Stepper Read
BSSBLIB.readDriverStatus.restype = ctypes.c_uint8
BSSBLIB.readCurrentPosition.restype = ctypes.c_int32
BSSBLIB.readEncoderPosition.restype = ctypes.c_int32
BSSBLIB.readCurrentSpeed.restype = ctypes.c_int32
BSSBLIB.readSensors.restype = ctypes.c_uint8
BSSBLIB.readEncoderPositionDeviation.restype = ctypes.c_int32

# Motion Planner
BSSBLIB.getMotionQueueStatus.restype = _QueueStatus
BSSBLIB.getMotionQueueStatus.argtypes = [ctypes.c_void_p]
BSSBLIB.mq_startMotion.restype = ctypes.c_bool
BSSBLIB.abortMotion.restype = ctypes.c_bool
BSSBLIB.clearMotionQueueReadback.restype = None
BSSBLIB.getMotionQueueReadback.restype = ctypes.c_bool

# Queue Item / Driver / Motion / Homing
BSSBLIB.writeQueueItem.restype = ctypes.c_bool
BSSBLIB.writeTestQueueItem.restype = ctypes.c_bool
BSSBLIB.mq_startTestQueueItem.restype = ctypes.c_bool
BSSBLIB.q_setAndInitDriver.restype = ctypes.c_bool
BSSBLIB.q_initDriver.restype = ctypes.c_bool
BSSBLIB.q_setMotion.restype = ctypes.c_bool
BSSBLIB.configureHoming.restype = ctypes.c_bool


# ==================================================================================== #
#                                    Defined Values                                    #
# ==================================================================================== #
class RampMode(enum.IntEnum):
    VELOCITY_MODE = 0x00
    POSITIONING_MODE = 0x01 << 2


class RampType(enum.IntEnum):
    HOLD_RAMP = 0x00
    TRAPEZOIDAL = 0x01
    S_CURVE = 0x02


class HomingMode(enum.IntEnum):
    IMMEDIATE = 0x00
    TORQUE = 0x01
    SENSOR = 0x02


class HomingSensor(enum.IntEnum):
    STOP_L = 0x00
    STOP_R = 0x01


class PositioningMode(enum.IntEnum):
    PM_RELATIVE = 0x00
    PM_ABSOLUTE = 0x01


class FollowMode(enum.IntEnum):
    INTERNAL = 0x00
    ENCODER = 0x01


# ==================================================================================== #
#                                        Structs                                       #
# ==================================================================================== #


class _QueueItemParameters(ctypes.Structure):
    _fields_ = [
        ("motion_type", ctypes.c_uint16),
        ("driver_num", ctypes.c_uint16),
        ("position", ctypes.c_int32),
        ("offset", ctypes.c_int32),
        ("homing_mode", ctypes.c_uint16),
        ("homing_sensor", ctypes.c_uint16),
        ("sensor_home_value", ctypes.c_uint16),
        ("ramp_type", ctypes.c_uint16),
        ("max_speed", ctypes.c_float),
        ("max_accel", ctypes.c_float),
        ("max_decel", ctypes.c_float),
        ("bow1", ctypes.c_float),
        ("bow2", ctypes.c_float),
        ("bow3", ctypes.c_float),
        ("bow4", ctypes.c_float),
        ("use_inverse_time", ctypes.c_uint16),
        ("time_ms", ctypes.c_uint32),
    ]


class QueueItemParameters(BaseModel):
    """
    Helper model to construct struct
    """

    motion_type: int = Field(default=0)
    driver_num: int = Field(default=0)
    position: int = Field(default=0)
    offset: int = Field(default=0)
    homing_mode: int = Field(default=0)
    homing_sensor: int = Field(default=0)
    sensor_home_value: int = Field(default=0)
    ramp_type: int = Field(default=0)
    max_speed: float = Field(default=0.0)
    max_accel: float = Field(default=0.0)
    max_decel: float = Field(default=0.0)
    bow1: float = Field(default=0.0)
    bow2: float = Field(default=0.0)
    bow3: float = Field(default=0.0)
    bow4: float = Field(default=0.0)
    use_inverse_time: int = Field(default=0)
    time_ms: int = Field(default=0)


class _DriverConfig(ctypes.Structure):
    _fields_ = [
        ("mstep_per_fs", ctypes.c_uint16),
        ("fs_per_rev", ctypes.c_uint16),
        ("unit_per_rev", ctypes.c_uint16),
        ("holding_current", ctypes.c_uint16),
        ("peak_rms_current", ctypes.c_uint16),
        ("stop_on_stall_enable", ctypes.c_bool),
        ("stop_on_stall_thresh", ctypes.c_uint16),
        ("cl_enable", ctypes.c_bool),
        ("cl_enable_pid", ctypes.c_bool),
        ("cl_enc_in_res", ctypes.c_uint16),
        ("cl_tolerance", ctypes.c_uint16),
        ("stealth_chop_thresh", ctypes.c_uint16),
    ]


class DriverConfig(BaseModel):
    """
    Helper model to construct struct
    """

    mstep_per_fs: int = Field(default=256)
    fs_per_rev: int = Field(default=200)
    unit_per_rev: int = Field(default=360)
    holding_current: int = Field(default=250)
    peak_rms_current: int = Field(default=500)
    stop_on_stall_enable: bool = Field(default=False)
    stop_on_stall_thresh: int = Field(default=100)
    cl_enable: bool = Field(default=False)
    cl_enable_pid: bool = Field(default=False)
    cl_enc_in_res: int = Field(default=4000)
    cl_tolerance: int = Field(default=500)
    stealth_chop_thresh: int = Field(default=180)


class _MotionConfig(ctypes.Structure):
    _fields_ = [
        ("allow_write_motion_when_busy", ctypes.c_uint16),
        ("reset_motion_conf_after_each_move", ctypes.c_uint16),
        ("pos_mode", ctypes.c_uint16),
        ("follow_mode", ctypes.c_uint16),
        ("ramp_mode", ctypes.c_uint16),
        ("ramp_type", ctypes.c_uint16),
        ("max_speed", ctypes.c_float),
        ("start_speed", ctypes.c_float),
        ("stop_speed", ctypes.c_float),
        ("break_speed", ctypes.c_float),
        ("max_accel", ctypes.c_float),
        ("max_decel", ctypes.c_float),
        ("start_accel", ctypes.c_float),
        ("final_decel", ctypes.c_float),
        ("bow1", ctypes.c_float),
        ("bow2", ctypes.c_float),
        ("bow3", ctypes.c_float),
        ("bow4", ctypes.c_float),
    ]


class MotionConfig(BaseModel):
    """
    Helper model to construct struct
    """

    allow_write_motion_when_busy: int = Field(default=0)
    reset_motion_conf_after_each_move: int = Field(default=0)
    pos_mode: int = Field(default=0)
    follow_mode: int = Field(default=0)
    ramp_mode: int = Field(default=4)
    ramp_type: int = Field(default=1)
    max_speed: float = Field(default=0.0)
    start_speed: float = Field(default=0.0)
    stop_speed: float = Field(default=0.0)
    break_speed: float = Field(default=0.0)
    max_accel: float = Field(default=0.0)
    max_decel: float = Field(default=0.0)
    start_accel: float = Field(default=0.0)
    final_decel: float = Field(default=0.0)
    bow1: float = Field(default=0.0)
    bow2: float = Field(default=0.0)
    bow3: float = Field(default=0.0)
    bow4: float = Field(default=0.0)


class _HomingConfig(ctypes.Structure):
    _fields_ = [
        ("homing_mode", ctypes.c_uint16),
        ("homing_sensor", ctypes.c_uint16),
        ("home_sensor_value", ctypes.c_uint16),
        ("homing_offset_units", ctypes.c_int32),
        ("homing_distance_units", ctypes.c_int32),
        ("max_speed", ctypes.c_float),
        ("max_accel", ctypes.c_float),
        ("max_decel", ctypes.c_float),
        ("homing_timeout_ms", ctypes.c_uint32),
    ]


class HomingConfig(BaseModel):
    """
    Helper model to construct struct
    """

    homing_mode: int = Field(default=0)
    homing_sensor: int = Field(default=0)
    home_sensor_value: int = Field(default=0)
    homing_offset_units: int = Field(default=0)
    homing_distance_units: int = Field(default=0)
    max_speed: float = Field(default=0.0)
    max_accel: float = Field(default=0.0)
    max_decel: float = Field(default=0.0)
    homing_timeout_ms: int = Field(default=10000)


# ==================================================================================== #
#                                         Board                                        #
# ==================================================================================== #


class _Callback(enum.Enum):
    CAN_TRANSMIT = enum.auto()
    CAN_RECEIVE = enum.auto()
    CAN_MUTEX = enum.auto()
    ASYNC_SLEEP = enum.auto()


class DriverStatus(enum.IntEnum):
    READY = 1
    BUSY = 2
    HOMING = 3
    VIBRATING = 4
    CTRL_NOT_INIT = 5
    MOTION_NOT_INIT = 6
    STALL = 7
    POS_ERR = 8
    STOPPED = 9
    COIL_OL = 10
    COIL_SHORT = 11
    OVERTEMP = 12
    TMC4361A_COMM_ERR = 13
    TMC5160_COMM_ERR = 14
    NO_COMM = 0xFFFF


class ExecCode(enum.IntEnum):
    SUCCESS = 1
    WRITFAIL = 2
    CTRL_NOT_INIT = 3
    MOTION_NOT_INIT = 4
    IS_FROZEN = 5
    IS_BUSY = 6
    BAD_SETTING = 7
    INVALID = 0xFFFF


class HomingCode(enum.IntEnum):
    SUCCESS = 1
    WRITE_FAIL = 2
    CTRL_NOT_INIT = 3
    MOTION_NOT_INIT = 4
    IS_FROZEN = 5
    IS_BUSY = 6
    TIMEOUT = 7
    MAX_PULSE_REACHED = 8
    FAILED_MIDWAY = 9
    INVALID = 0xFFFF


class Response(enum.IntEnum):
    SUCCESS = 0x0001
    INVALID_INPUT = 0x1000
    VALUE_OUT_OF_BOUNDS = 0x1001
    WRITE_FAIL = 0x1002
    READ_FAIL = 0x1003
    DRIVER_BUSY = 0x1004
    DRIVER_IS_NULL = 0x1005
    USED_BY_QUEUE = 0x1006
    UNKNOWN = 0x1007
    INVALID = 0xFFFF


class MotionQueueRes(enum.IntEnum):
    SUCCESS = 1
    CONDITIONAL_SUCCESS = 2
    FAILED_MIDWAY = 3
    INVALID_MOTION = 4
    EMPTY_QUEUE = 5
    EXCEED_MAX_CONCURRENT = 6
    REPEATED_ASSIGNMENT = 7
    BAD_PARAMETERS = 8
    STEPPERS_NOT_READY = 9
    FAILED_TO_FETCH = 0xFFFE
    INVALID = 0xFFFF


class BoardAPI:
    def __init__(self, board_id: int):
        self._callbacks = {}

        self.board_id = board_id
        self.handle = self._create_board_handle(self.board_id)

    def _create_board_handle(self, board_id: int):
        handle = BSSBLIB.bsSb_create(board_id)

        return handle

    def deinit(self):
        """
        Deinitialize bsSb object
        """
        BSSBLIB.bsSb_destroy(self.handle)

    def reset(self, init_driver=False):
        """
        Resets the state of all drivers on the board.

        Args:
            init_driver (bool, optional):
                If True, automatically re-initialize all drivers.
                Should be used when the SD card is in use to ensure proper driver configuration.

        Returns:
            None
        """
        for i in range(5):
            self.emergency_stop(i)

        for i in range(5):
            self.clear_queued_readback(i)

        if init_driver:
            for i in range(5):
                self.q_init_driver(i)

            for i in range(5):
                self.get_queue_readback(i)

        self.clear_motion_queue_readback()

    # ==================================================================================== #
    #                                       Override                                       #
    # ==================================================================================== #

    def override_CANTransmit(self, func: Callable[[int, int, int], bool]):
        """
        Override the default CAN transmit function.

        Args:
            func (Callable[[int, int, int], int]):
                Expected signature:
                def myCANTransmit(identifier, data, length)
                    return can_device.transmit(identifier, data, length)
        Returns:
            None
        """
        C_CANTransmit = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint8)

        cb = C_CANTransmit(func)
        self._callbacks[_Callback.CAN_TRANSMIT] = cb

        BSSBLIB.setCANTransmit(self.handle, cb)

    def override_CANReceive(self, func: Callable[[int, int], int]):
        """
        Overrides the default CAN receive function.

        Args:
            func (Callable[[int, int], int]):
                Expected signature:
                def myCANReceive(buf_ptr, size):
                    buffer = []
                    num = can_device.receive(buffer, size, 50)
                    for i in range(num):
                        buf_ptr[i] = buffer[i]
                    return num

        Returns:
            None
        """
        C_CANReceive = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int)

        cb = C_CANReceive(func)
        self._callbacks[_Callback.CAN_RECEIVE] = cb

        BSSBLIB.setCANReceive(self.handle, cb)

    def override_CANMutex(self, func: Callable[[bool], None]):
        """
        Overrides the default CAN Mutex function.
        *Can be left unset*

        Args:
            func (Callable[[bool], None]):
                Expected signature:
                import asyncio
                can_bus_mutex = asyncio.Lock()

                def myCANMutex(acquire: bool):
                    can_bus_mutex.acquire() if acquire else can_bus_mutex.release()

        Returns:
            None
        """
        C_CANMutex = ctypes.CFUNCTYPE(None, ctypes.c_bool)

        cb = C_CANMutex(func)
        self._callbacks[_Callback.CAN_MUTEX] = cb

        BSSBLIB.setCANMutex(self.handle, cb)

    def override_asyncSleep(self, func: Callable[[], None]):
        """
        Overrides the default sleep function.
        *Asynchronous depending on usage*

        Args:
            func (Callable[[], None]):
                Expected signature:
                import time
                def myAsyncSleep():
                    time.sleep(0.001)

                import asyncio
                async def myAsyncSleep()
                    await asyncio.sleep(0.001)

        Returns:
            None
        """
        # Wrap async functions in a synchronous callable for CFUNCTYPE
        if asyncio.iscoroutinefunction(func):

            def wrapper():
                asyncio.run(func())

        else:
            wrapper = func

        C_AsyncSleep = ctypes.CFUNCTYPE(None)

        cb = C_AsyncSleep(wrapper)
        self._callbacks[_Callback.ASYNC_SLEEP] = cb

        BSSBLIB.setAsyncSleep(self.handle, cb)

    # ==================================================================================== #
    #                                      Board Info                                      #
    # ==================================================================================== #

    def read_firmware_version(self):
        """
        Returns current firmware version

        """
        content_ptr = ctypes.c_char_p()
        success = BSSBLIB.readFirmwareVersion(self.handle, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    # ==================================================================================== #
    #                                        SD Card                                       #
    # ==================================================================================== #

    def read_driver_config(self, driver_num: int):
        """
        Reads file stored at `config/..` on the SD card

        Args:
            driver_num (int):
                values from 0-4

        Returns:
            content (str):
                If successful
        """
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readDriverConfig(self.handle, driver_num, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def read_motion_config(self, motion_index: int):
        """
        Reads motion stored at `motion/..` on the SD card

        Args:
            motion_index (int):

        Returns:
            content (str):
                If successful
        """
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readMotionConfig(self.handle, motion_index, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def read_sequence_config(self, sequence_index: int):
        """
        Reads sequence stored at `sequence/..` on the SD card

        Args:
            sequence_index (int):
                values from 0-15

        Returns:
            content (str):
                If successful
        """
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readSequenceConfig(self.handle, sequence_index, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def read_buffer(self):
        """
        Reads current string held by `buffer`

        Returns:
            content (str):
                If successful
        """
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readBuffer(self.handle, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def read_path(self):
        """
        Reads current string held by `path`

        Returns:
            content (str):
                If successful
        """
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readPath(self.handle, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def read_file_at_path(self, path: str):
        """
        Reads file at `path` on the SD card

        Args:
            path (str):
                - path (excluding root)
                - eg: `config/4`

        Returns:
            successful (bool):
        """
        path_ptr = ctypes.c_char_p(path.encode("utf-8"))
        content_ptr = ctypes.c_char_p()

        success = BSSBLIB.readFileAtPath(self.handle, path_ptr, ctypes.byref(content_ptr))
        if success and content_ptr.value is not None:
            content = content_ptr.value.decode()
            # Free memory allocated by strdup
            libc = ctypes.CDLL("libc.so.6")
            libc.free(content_ptr)
            return content

        return None

    def delete_file_at_path(self, path: str) -> bool:
        """
        Deletes file at `path` on the SD card

        Args:
            path (str):
                - path (excluding root)
                - eg: `config/4`

        Returns:
            successful (bool):
        """
        path_ptr = ctypes.c_char_p(path.encode("utf-8"))

        success = BSSBLIB.deleteFileAtPath(self.handle, path_ptr)

        return success

    def write_to_file_at_path(self, path: str, content: str) -> bool:
        """
        Writes to file at `path` on the SD card

        Args:
            path (str):
                - path (excluding root)
                - eg: `config/4`
            content (str):

        Returns:
            successful (bool):
        """
        path_ptr = ctypes.c_char_p(path.encode("utf-8"))
        content_ptr = ctypes.c_char_p(content.encode("utf-8"))

        success = BSSBLIB.writeToFileAtPath(self.handle, path_ptr, content_ptr)

        return success

    # ==================================================================================== #
    #                                   Readback (Driver)                                  #
    # ==================================================================================== #

    def clear_queued_readback(self, driver_num: int):
        """
        Clears result of last queued operation
        """
        BSSBLIB.clearQueuedReadback(self.handle, driver_num)

    def get_queue_readback(self, driver_num: int):
        """
        Gets readback from previous queued operation. **Blocks until completion / query timeout**

        Args:
            driver_num (int):
                values 0-4

        Returns:
            response_code (int):
                - `q_move_preloaded` -> `ExecCode`
                - `q_move_stepper_vibration` -> `ExecCode`
                - `q_move_stepper_inverse_time` -> `ExecCode`
                - `q_move_stepper_homing` -> `HomingCode`
                - `q_move_stepper` -> `ExecCode`
                - `q_set_motion` -> `Response`
                - `q_init_driver` -> `Response`
                - `q_set_and_init_driver` -> `Response`


        See Also:
            `q_move_preloaded`, `q_move_stepper_vibration`, `q_move_stepper_inverse_time`, 
            `q_move_stepper_homing`, `q_move_stepper`, `q_set_motion`, `q_init_driver`, `q_set_and_init_driver`
        """
        response = ctypes.c_uint16()

        success = BSSBLIB.getQueueReadback(self.handle, driver_num, ctypes.byref(response))
        if success and response.value is not None:
            return response.value

        return None

    @staticmethod
    def queue_readback_to_homing_code(response_code: int):
        """
        Maps response_code to `HomingCode`
        """
        try:
            return HomingCode(response_code)
        except ValueError:
            return HomingCode.INVALID

    @staticmethod
    def queue_readback_to_exec_code(response_code: int):
        """
        Maps response_code to `ExecCode`
        """
        try:
            return ExecCode(response_code)
        except ValueError:
            return ExecCode.INVALID

    @staticmethod
    def queue_readback_to_response(response_code: int):
        """
        Maps response_code to `Response`
        """
        try:
            return Response(response_code)
        except ValueError:
            return Response.INVALID

    # ==================================================================================== #
    #                                     Move Commands                                    #
    # ==================================================================================== #

    def q_move_preloaded(self, index: int) -> tuple[bool, int]:
        """
        Puts `preloaded motion` into driver queue

        Args:
            index (int):
                As defined in `motion/_indexMap` on the SD card

        Returns:
            tuple (tuple[bool, int]):
                successfully put into queue, driver number

        Note:
            `PreloadPutQueueRes` is not exposed. On failure, please check motion type.
            Only `move` commands are allowed

        See Also:
            `get_queue_readback`
        """
        driver_number = ctypes.c_int8()

        success = BSSBLIB.q_movePreloaded(self.handle, index, ctypes.byref(driver_number))

        return success, driver_number.value

    def q_move_stepper(self, driver_num: int, position_units: int) -> bool:
        """
        Puts `move` command into driver queue

        Args:
            driver_num (int):
                values 0-4
            position_units (int):
                - If ramp_mode is set to `RampMode.VELOCITY`, value represents RPM
                - Else, it represents units as defined by `units_per_rev` in driver config

        Returns:
            successful (bool):
                successfully put into queue

        See Also:
            `q_set_motion`, `get_queue_readback`
        """
        return BSSBLIB.q_moveStepper(self.handle, driver_num, ctypes.c_int32(position_units))

    def q_move_stepper_homing(self, driver_num: int) -> bool:
        """
        Puts `move homing` into driver queue

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):
                successfully put into queue

        See Also:
            `configure_homing`, `get_queue_readback`
        """
        return BSSBLIB.q_moveStepper_homing(self.handle, driver_num)

    def q_move_stepper_inverse_time(self, driver_num: int, position_units: int, time_ms: int) -> bool:
        """
        Puts `move (inverse time)` command into driver queue

        Args:
            driver_num (int):
                values 0-4
            position_units (int):
                units as defined by `units_per_rev` in driver config
            time_ms (int):
                time to complete motion

        Returns:
            successful (bool):
                successfully put into queue

        Note:
            Automatically sets to RampType.TRAPEZOIDAL

        See Also:
            `get_queue_readback`
        """
        return BSSBLIB.q_moveStepper(self.handle, driver_num, ctypes.c_int32(position_units), ctypes.c_uint32(time_ms))

    def q_move_stepper_vibration(
        self, driver_num: int, position_units: int, iterations: int, diminishing_factor: float, loop: bool
    ) -> bool:
        """
        Puts `move vibration` into driver queue

        Args:
            driver_num (int):
                values 0-4
            position_units (int):
                units as defined by `units_per_rev` in driver config
            diminishing_factor (float):
                - gradual decrease in amplitude
                - Eg: `position_units` is set to 1 and `diminishing_factor` is set to 0.9.
                    Vibration will go from `1 -> 0.9 -> 0.81...`
            loop (bool):
                - set to True if vibration should not stop.
                - **If set to True, do not call `get_queue_readback` before `ramp_stop`**

        Returns:
            successful (bool):
                successfully put into queue

        See Also:
            `get_queue_readback`
        """
        return BSSBLIB.q_moveStepper_vibration(
            self.handle,
            driver_num,
            ctypes.c_int32(position_units),
            ctypes.c_uint16(iterations),
            ctypes.c_float(diminishing_factor),
            ctypes.c_bool(loop),
        )

    # ==================================================================================== #
    #                                     Control Word                                     #
    # ==================================================================================== #

    def ramp_stop(self, driver_num: int) -> bool:
        """
        Stops driver gracefully according to set deceleration parameters.
        If driver is used by motion queue, `emergency_stop` will be called on all drivers
        involved in the motion queue

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):
        """
        return BSSBLIB.rampStop(self.handle, driver_num)

    def emergency_stop(self, driver_num: int) -> bool:
        """
        Stops driver immediately.
        If driver is used by motion queue, `emergency_stop` will be called on all drivers
        involved in the motion queue

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):
        """
        return BSSBLIB.emergencyStop(self.handle, driver_num)

    def reset_position(self, driver_num: int) -> bool:
        """
        Resets internal and encoder position to 0

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):

        Note:
            Can only be called when driver is not `BUSY`
        """
        return BSSBLIB.resetPosition(self.handle, driver_num)

    def enable_driver(self, driver_num: int) -> bool:
        """
        Powers driver. Automatically called by move commands

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):

        Note:
            Can only be called when driver is not `BUSY`
        """
        return BSSBLIB.enableDriver(self.handle, driver_num)

    def release_driver(self, driver_num: int) -> bool:
        """
        Un-powers driver

        Args:
            driver_num (int):
                values 0-4

        Returns:
            successful (bool):

        Note:
            Can only be called when driver is not `BUSY`
        """
        return BSSBLIB.releaseDriver(self.handle, driver_num)

    # ==================================================================================== #
    #                                     Stepper Read                                     #
    # ==================================================================================== #

    def read_driver_status(self, driver_num: int):
        """
        Fetches driver status

        Args:
            driver_num (int):
                values 0-4

        Returns:
            DriverStatus
        """
        try:
            return DriverStatus(BSSBLIB.readDriverStatus(self.handle, driver_num))

        except ValueError:
            return DriverStatus.NO_COMM

    def read_current_position(self, driver_num: int) -> int:
        """
        Fetches current internal position (unit: user units)

        Args:
            driver_num (int):
                values 0-4

        Returns:
            position (int):
        """
        return BSSBLIB.readCurrentPosition(self.handle, driver_num)

    def read_encoder_position(self, driver_num: int) -> int:
        """
        Fetches current encoder position (unit: user units)

        Args:
            driver_num (int):
                values 0-4

        Returns:
            position (int):
        """
        return BSSBLIB.readEncoderPosition(self.handle, driver_num)

    def read_current_speed(self, driver_num: int) -> int:
        """
        Fetches current speed (unit: RPM)

        Args:
            driver_num (int):
                values 0-4

        Returns:
            RPM (int):
        """
        return BSSBLIB.readCurrentSpeed(self.handle, driver_num)

    def read_sensors(self, driver_num: int) -> int:
        """
        Fetches current readings of `STOP_L` and `STOP_R` sensors on the corresponding driver

        Args:
            driver_num (int):
                values 0-4

        Returns:
            sensor_reading (int):
                - bit0 is `STOP_L`
                - bit1 is `STOP_R`
                - Rest ignore

        """
        return BSSBLIB.readSensors(self.handle, driver_num)

    def read_encoder_position_deviation(self, driver_num: int) -> int:
        """
        Fetches deviation between internal position and encoder position (unit: microsteps)

        Args:
            driver_num (int):
                values 0-4

        Returns:
            deviation (int):
        """
        return BSSBLIB.readEncoderPositionDeviation(self.handle, driver_num)

    # ==================================================================================== #
    #                                    Motion Planner                                    #
    # ==================================================================================== #

    def get_motion_queue_status(self) -> tuple[bool, list[int]]:
        """
        Fetches status of last called motion queue with `mq_start_motion`

        Returns:
            Tuple
            bool, List[int, int]:
                A tuple containing:
                - is_executing (bool): Whether the motion queue is currently executing.
                - [motion_num, sequence_num] (list[int, int]):
                  The active motion number and sequence number.
        """
        sts = BSSBLIB.getMotionQueueStatus(self.handle)

        return sts.is_executing, [sts.motion_num, sts.sequence_num]

    def mq_start_motion(self, motion_num: int) -> bool:
        """
        Places the sequence of motions as defined in `sequence/<motion_num>` on the SD card into
        the motion queue

        Args:
            motion_num (int):
                Defined in `sequence/<motion_num>`

        Returns:
            successful (bool):
                successfully put into motion queue

        See Also:
            `get_motion_queue_status`, `get_motion_queue_readback`
        """
        return BSSBLIB.mq_startMotion(self.handle, motion_num)

    def abort_motion(self) -> bool:
        """
        Aborts current motion queue

        Returns:
            successful (bool):

        Note:
            Clear motion queue readback after
        """
        return BSSBLIB.abortMotion(self.handle)

    def clear_motion_queue_readback(self):
        """
        Clears result of last motion queue
        """
        BSSBLIB.clearMotionQueueReadback(self.handle)

    def get_motion_queue_readback(self):
        """
        Fetches the results of last sequence of motions called with `mq_start_motion`. **Blocks until completion / query timeout**

        Returns:
            MotionQueueRes
        """
        response = ctypes.c_uint16()

        success = BSSBLIB.getMotionQueueReadback(self.handle, ctypes.byref(response))

        if not success:
            return MotionQueueRes.FAILED_TO_FETCH
        try:
            return MotionQueueRes(response.value)
        except ValueError:
            return MotionQueueRes.INVALID

    def write_queue_item(self, motion_num: int, item_num: int, sequence_num: int, params: QueueItemParameters) -> bool:
        """
        Overrides the motion parameters of the selected `motion_num` and `item_num` with `sequence_num` and `QueueItemParameters`

        Args:
            motion_num (int):
                equivalent to `sequence/<motion_num>` on the SD card
            item_num (int):
                equivalent to the line number in `sequence/<motion_num>` on the SD card
            sequence_num (int):
                equivalent to **`sequence_num`**=`params` of the line number
            params (QueueItemParameters):
                equivalent to `sequence_num`=**`params`** of the line number

        Returns:
            successful (bool):
        """
        struct = bsSb._param_to_struct(params, _QueueItemParameters)

        return BSSBLIB.writeQueueItem(
            self.handle,
            ctypes.c_uint16(motion_num),
            ctypes.c_uint16(item_num),
            ctypes.c_uint16(sequence_num),
            ctypes.byref(struct),
        )

    def start_test_queue_item(self) -> bool:
        """
        Starts test queue item

        Returns:
            successful (bool):
                successfully put into motion queue
        """
        return BSSBLIB.mq_startTestQueueItem(self.handle)

    def write_test_queue_item(self, params: QueueItemParameters) -> bool:
        """
        Writes `QueueItemParameters` into test queue item

        Returns:
            successful (bool):
        """
        struct = bsSb._param_to_struct(params, _QueueItemParameters)

        return BSSBLIB.writeTestQueueItem(ctypes.byref(struct))

    # ==================================================================================== #
    #                                        Driver                                        #
    # ==================================================================================== #

    def q_set_and_init_driver(self, driver_num: int, params: DriverConfig) -> bool:
        """
        Puts set and init driver into driver queue

        Args:
            driver_num (int):
                Values 0-4
            params (DriverConfig):
                See documentation

        Returns:
            successful (bool):
                Sucessfully put into queue

        See Also:
            `get_queued_readback`
        """
        struct = bsSb._param_to_struct(params, _DriverConfig)

        return BSSBLIB.q_setAndInitDriver(
            self.handle,
            driver_num,
            ctypes.byref(struct),
        )

    def q_init_driver(self, driver_num: int) -> bool:
        """
        Puts init driver into driver queue. Best used with config set on SD Card

        Args:
            driver_num (int):
                Values 0-4

        Returns:
            successful (bool):
                Sucessfully put into queue

        See Also:
            `get_queued_readback`
        """
        return BSSBLIB.q_initDriver(self.handle, driver_num)

    # ==================================================================================== #
    #                                        Motion                                        #
    # ==================================================================================== #

    def q_set_motion(self, driver_num: int, params: MotionConfig) -> bool:
        """
        Puts set motion into driver queue.

        Args:
            driver_num (int):
                Values 0-4
            params (MotionConfig):
                See documentation

        Returns:
            successful (bool):
                Sucessfully put into queue

        See Also:
            `get_queued_readback`

        """
        struct = bsSb._param_to_struct(params, _MotionConfig)

        return BSSBLIB.q_setMotion(
            self.handle,
            driver_num,
            ctypes.byref(struct),
        )

    # ==================================================================================== #
    #                                        Homing                                        #
    # ==================================================================================== #

    def configure_homing(self, driver_num: int, params: HomingConfig) -> bool:
        """
        Configures parameters for homing modes

        Args:
            driver_num (int):
                values 0-4
            params (HomingConfig):
                See documentation
                
        Returns:
            successful (bool):
        """
        struct = bsSb._param_to_struct(params, _HomingConfig)

        return BSSBLIB.configureHoming(
            self.handle,
            driver_num,
            ctypes.byref(struct),
        )

    # ==================================================================================== #
    #                                      Py Utility                                      #
    # ==================================================================================== #
    T = TypeVar("T", bound=ctypes.Structure)

    @staticmethod
    def _param_to_struct(model: BaseModel, struct_cls: Type[T]) -> T:
        """
        Converts a Pydantic BaseModel to a ctypes.Structure instance.

        Args:
            model (BaseModel): The Pydantic model containing parameter values.
            struct_cls (Type[T]): The ctypes.Structure class to map to.

        Returns:
            T: An instance of the ctypes.Structure populated with values from the model.
        """
        struct = struct_cls()  # create empty ctypes struct
        for field_name, _ in struct_cls._fields_:
            if hasattr(model, field_name):
                setattr(struct, field_name, getattr(model, field_name))
            else:
                raise AttributeError(f"Model '{type(model).__name__}' has no field '{field_name}'")
        return struct
