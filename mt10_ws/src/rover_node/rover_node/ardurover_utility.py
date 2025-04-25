import sys
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class Vehicle:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        """
        Initialize the vehicle connection.

        Args:
            port (str): The port to connect to the vehicle.
            baud (int): The baud rate for the connection.
        """
        self.port = port
        self.baud = baud
        self.rover = None
        self.heartbeat = False

    def init_connection(self):
        """
        Initialize the connection to the vehicle.

        Returns:
            mavutil.mavlink_connection: The MAVLink connection object.
        """
        self.rover = mavutil.mavlink_connection(self.port, self.baud)
        self.rover.wait_heartbeat()
        return self.rover

    def arm(self):
        """
        Arm the vehicle.
        """
        self.rover.mav.command_long_send(
            self.rover.target_system,
            self.rover.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 21196, 0, 0, 0, 0, 0
        )
        print("Arming the vehicle")
        self.rover.motors_armed_wait()
        print("Vehicle armed")
        time.sleep(2)

    def disarm(self):
        """
        Disarm the vehicle.
        """
        self.rover.mav.command_long_send(
            self.rover.target_system,
            self.rover.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 21196, 0, 0, 0, 0, 0
        )
        self.rover.motors_disarmed_wait()
        time.sleep(2)

    def reboot(self):
        """
        Reboot the vehicle's autopilot.
        """
        print("Starting system reboot")
        self.rover.reboot_autopilot()

    def set_relay(self, rgb):
        """
        Set the relay to a specific RGB value.

        Args:
            rgb (int): The RGB value to set.
        """
        if rgb == 'red':
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                2, 0, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                1, 1, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                0, 1, 0, 0, 0, 0, 0
            )

        elif rgb == 'green':
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
                0,
                1, 15, 1, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                2, 1, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                0, 1, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
        elif rgb == 'blue':
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                1, 1, 0, 0, 0, 0, 0
            )
            self.rover.wait_heartbeat()
            self.rover.mav.command_long_send(
                self.rover.target_system,
                self.rover.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                0,
                2, 1, 0, 0, 0, 0, 0
            )   

    def set_rc_channel_pwm(self, pitch=1500, throttle=1500, base=1500, link1=1500, link2=1500, d1=1500, d2=1500, claw=1500):
        """
        Set RC channel PWM values.

        Args:
            pitch (int): PWM value for pitch.
            throttle (int): PWM value for throttle.
            base (int): PWM value for base.
            link1 (int): PWM value for link1.
            link2 (int): PWM value for link2.
            d1 (int): PWM value for d1.
            d2 (int): PWM value for d2.
            claw (int): PWM value for claw.
        """
        command = [pitch, base, throttle, link1, link2, d1, d2, claw]
        rc_channel_values = [65535 for _ in range(8)]

        for i in range(8):
            rc_channel_values[i] = command[i]

        self.rover.mav.rc_channels_override_send(
            self.rover.target_system,
            self.rover.target_component,
            *rc_channel_values
        )

    def get_ahrs2(self):
        """
        Get AHRS2 data from the vehicle.

        Returns:
            dict: AHRS2 data.
        """
        msg = self.rover.recv_match(type='AHRS2', blocking=True).to_dict()
        return msg

    def get_gps(self):
        """
        Get GPS data from the vehicle.

        Returns:
            dict: GPS data.
        """
        msg = self.rover.recv_match(type='GPS_RAW_INT', blocking=True).to_dict()
        return msg

    def get_current_mode(self):
        """
        Get the current mode of the vehicle.

        Returns:
            str: Current mode.
        """
        mode_id = self.rover.wait_heartbeat().custom_mode
        current_mode = list(self.rover.mode_mapping().keys())[list(self.rover.mode_mapping().values()).index(mode_id)]
        return current_mode

    def set_mode(self, mode):
        """
        Set the mode of the vehicle.

        Args:
            mode (str): The mode to set (MANUAL, SIMPLE, AUTO).
        """
        mode_id = self.rover.mode_mapping()[mode]
        while not self.rover.wait_heartbeat().custom_mode == mode_id:
            self.rover.set_mode(mode_id)
            self.acknowledge_mode_change(mode)

    def set_manual_mode(self):
        """
        Set the vehicle to MANUAL mode.
        """
        print("Setting mode to MANUAL")
        self.set_mode("MANUAL")

    def set_simple_mode(self):
        """
        Set the vehicle to SIMPLE mode.
        """
        print("Setting mode to SIMPLE")
        self.set_mode("SIMPLE")

    def set_auto_mode(self):
        """
        Set the vehicle to AUTO mode.
        """
        print("Setting mode to AUTO")
        self.set_mode("AUTO")

    def acknowledge_mode_change(self, mode):
        """
        Acknowledge the mode change.

        Args:
            mode (str): The mode that was set.
        """
        ack_msg = self.rover.recv_match(type="COMMAND_ACK", blocking=True)
        if ack_msg.command in (176, 11):
            if ack_msg.result == 0:
                print(f"Mode change to {mode}")
            result = mavutil.mavlink.enums['MAV_RESULT'][ack_msg.result]
            print(result.name)