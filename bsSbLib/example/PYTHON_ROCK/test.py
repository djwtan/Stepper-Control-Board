import time
import serial
import ws_can_usb_a
import bsSbAPI

if __name__ == "__main__":
    # ==================================================================================== #
    #                                Setup Waveshare Driver                                #
    # ==================================================================================== #

    # ws_can_usb_a.DEBUG = True

    t = serial.Serial("/dev/ttyUSB0", 2000000)

    can_settings = ws_can_usb_a.CANSettings(
        protocol=ws_can_usb_a.S_Protocol.VARIABLE,
        frame_type=ws_can_usb_a.S_FrameType.STANDARD,
        frame_format=ws_can_usb_a.S_FrameFormat.DATA,
        can_mode=ws_can_usb_a.S_CANMode.NORMAL,
        baudrate=ws_can_usb_a.S_BaudRate.BR_500K,
    )

    can_device = ws_can_usb_a.Waveshare_USBCAN_A(t, can_settings)

    # ==================================================================================== #
    #                                  Override Functions                                  #
    # ==================================================================================== #

    board = bsSbAPI.BoardAPI(1)

    def myCANTransmit(id, data, length):
        return can_device.transmit(id, data, length)

    def myCANReceive(buf_ptr, size):
        buffer = []
        num = can_device.receive(buffer, size, 50)

        for i in range(num):
            buf_ptr[i] = buffer[i]

        return num

    def myAsyncSleep():
        time.sleep(0.001)

    board.override_CANTransmit(myCANTransmit)
    board.override_CANReceive(myCANReceive)
    board.override_asyncSleep(myAsyncSleep)

    board.reset(init_driver=True)

    # ==================================================================================== #
    #                                        Sd Card                                       #
    # ==================================================================================== #

    # print(board.read_firmware_version())
    # print(board.read_driver_config(2))
    # print(board.read_motion_config(2))
    # print(board.read_sequence_config(2))
    # print(board.read_buffer())
    # print(board.read_path())

    # print(board.write_to_file_at_path("hello", "a\nb\nSurprise"))
    # print(board.read_file_at_path("hello"))
    # print(board.delete_file_at_path("hello"))
    # print(board.read_file_at_path("hello"))

    # ==================================================================================== #
    #                                Pre-Configured Motions                                #
    # ==================================================================================== #

    # board.sync_and_reinit()

    # for i in range(50):

    #     success0, drv0 = board.q_move_preloaded(7 if i % 2 else 8)
    #     success1, drv1 = board.q_move_preloaded(13 if i % 2 else 14)

    #     if success0:
    #         res_code = board.get_queue_readback(drv0)
    #         if res_code is not None:
    #             print(bsSbAPI.queue_readback_to_homing_code(res_code))
    #     if success1:
    #         res_code = board.get_queue_readback(drv1)
    #         if res_code is not None:
    #             print(bsSbAPI.queue_readback_to_homing_code(res_code))

    # ==================================================================================== #
    #                                       Sequence                                       #
    # ==================================================================================== #

    while True:
        if board.mq_start_motion(0) and board.get_motion_queue_readback() == bsSbAPI.MotionQueueRes.SUCCESS:
            while board.mq_start_motion(1) and board.get_motion_queue_readback() == bsSbAPI.MotionQueueRes.SUCCESS:
                pass

            # time.sleep(0.1)
            # executing = 1
            # while executing == 1:
            #     res = board.get_motion_queue_status()
            #     executing, _ = res
            #     print(res)


    # ==================================================================================== #
    #                                        Manual                                        #
    # ==================================================================================== #
    # driver_num = 0
    # driver_config = bsSbAPI.DriverConfig(
    #     holding_current=500,
    #     peak_rms_current=3000,
    #     unit_per_rev=360,
    #     stealth_chop_thresh=400,
    # )

    # print(board.q_set_and_init_driver(driver_num, driver_config))
    # res_code = board.get_queue_readback(driver_num)

    # if res_code is not None:
    #     print(f"Init Driver -> {board.queue_readback_to_response(res_code)}")

    # motion_config = bsSbAPI.MotionConfig(
    #     max_speed=5.0,
    #     max_accel=5,
    #     max_decel=5,
    #     pos_mode=bsSbAPI.PositioningMode.PM_RELATIVE,
    # )

    # print(board.q_set_motion(driver_num, motion_config))
    # res_code = board.get_queue_readback(driver_num)

    # if res_code is not None:
    #     print(f"Init Motion -> {board.queue_readback_to_response(res_code)}")

    # print(board.q_move_stepper(driver_num, 360))
    # res_code = board.get_queue_readback(driver_num)

    # if res_code is not None:
    #     print(f"Move -> {board.queue_readback_to_response(res_code)}")

    # ------------------------------------------------------------------------------------ #

    board.deinit()
