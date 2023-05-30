from pymavlink import mavutil

class GroundControlChannel:
    def __init__(self, device:str, baud:int):
        if "udpin" in device:
            self.connection = mavutil.mavlink_connection(device=device, baudrate=baud)
        elif "/dev/tty" in device:
            self.connection = mavutil.mavserial(device=device, baud=baud)
    
    def wait_heartbeat(self):
        return self.connection.wait_heartbeat()

    def recv_match(self, type=None):
        return self.connection.recv_match(type=type)        


class VxChannel:
    def __init__(self,device:str, baud:int):
        if "udpout" in device:
            self.connection = mavutil.mavlink_connection(device=device, baudrate=baud)
        elif "/dev/tty" in device:
            self.connection = mavutil.mavserial(device=device, baud=baud)
        
    def send_hearbeat(self):
        self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        
    def send_gps(self,latitude,longitude,depth):
        """
        Latitude & Longitude (WGS84), in degrees * 1E7
        Depth, in m (positive for up)
        """
        self.connection.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        8 | 16 | 32,
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        latitude,  # Latitude (WGS84), in degrees * 1E7
        longitude,  # Longitude (WGS84), in degrees * 1E7
        depth,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        0   # Number of satellites visible
        )

    def send_battery(self):
        """
        Line 167770 in generated_messages.py
              
        Battery information. Updates GCS with flight controller battery
        status. Smart batteries also use this message, but may
        additionally send SMART_BATTERY_INFO.

        id                        : Battery ID (type:uint8_t)
        battery_function          : Function of the battery (type:uint8_t, values:MAV_BATTERY_FUNCTION)
        type                      : Type (chemistry) of the battery (type:uint8_t, values:MAV_BATTERY_TYPE)
        temperature               : Temperature of the battery. INT16_MAX for unknown temperature. [cdegC] (type:int16_t)
        voltages                  : Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1). [mV] (type:uint16_t)
        current_battery           : Battery current, -1: autopilot does not measure the current [cA] (type:int16_t)
        current_consumed          : Consumed charge, -1: autopilot does not provide consumption estimate [mAh] (type:int32_t)
        energy_consumed           : Consumed energy, -1: autopilot does not provide energy consumption estimate [hJ] (type:int32_t)
        battery_remaining         : Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery. [%] (type:int8_t)
        """
        self.connection.mav.battery_status_send(
            0,
            0,
            0,
            0,
            (0,0,0,0,0,0,0,0,0,0),
            0,
            0,
            0,
            0
            )
        
    def send_attitude(self, roll, pitch, yaw):
        """
        The attitude in the aeronautical frame (right-handed, Z-down, Y-right,
        X-front, ZYX, intrinsic).

        time_boot_ms              : Timestamp (time since system boot). [ms] (type:uint32_t)
        roll                      : Roll angle (-pi..+pi) [rad] (type:float)
        pitch                     : Pitch angle (-pi..+pi) [rad] (type:float)
        yaw                       : Yaw angle (-pi..+pi) [rad] (type:float)
        rollspeed                 : Roll angular speed [rad/s] (type:float)
        pitchspeed                : Pitch angular speed [rad/s] (type:float)
        yawspeed                  : Yaw angular speed [rad/s] (type:float)

        """
        self.connection.mav.attitude_send(
            0,
            roll,
            pitch,
            yaw,
            0,
            0,
            0
        )

