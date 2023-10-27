
import socket

from dronekit import connect,VehicleMode, LocationGlobalRelative,APIException
from pymavlink import mavutil
import time
import socket
import math
import argparse

def connectMyCopter():
        parser=argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args=parser.parse_args()

        connection_string=args.connect
        baud_rate=57600
        vehicle=connect(connection_string,baud=baud_rate,wait_ready=True)
        return vehicle

def arm():
        vehicle.mode=VehicleMode('GUIDED')
        vehicle.armed=True
        while  vehicle.armed==False:
                print('waiting to be armed')
                time.sleep(1)
        print('Drone is now armed')
        print('Drone Starting')
        return None


def unarm():
        vehicle.mode=VehicleMode('LAND')
        vehicle.armed=False
        return None




def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)


def keyboard():
    speed = 0.5
    up = 1
    vehicle.simple_takeoff(2)
    time.sleep(10)
    samay = time.time()
    while True and time.time()-samay<120:
        data = input("Input do bsdk?...")
        if data == 'b':
            break
        elif data=='w':
            second = time.time()
            while time.time()-second <5:
                send_ned_velocity(speed, 0, 0)
            send_ned_velocity(0,0,0)
        elif data=='s':
            second = time.time()
            while time.time()-second <5:
                send_ned_velocity(-1*speed, 0, 0)
            send_ned_velocity(0,0,0)


        elif data=='a':
            second = time.time()
            while time.time()-second <5:
                send_ned_velocity(0, -1*speed, 0)
            send_ned_velocity(0,0,0)


        elif data=='d':
            second = time.time()
            while time.time()-second <5:
                send_ned_velocity(0, speed, 0)
            send_ned_velocity(0,0,0)


        elif data=='h':
            for i in range(10):
                send_ned_velocity(0, 0, -1*up) 
            send_ned_velocity(0,0,0)


        elif data=='l':
            for i in range(10):
                send_ned_velocity(0, 0, up)
            send_ned_velocity(0,0,0)


vehicle=connectMyCopter()
arm()

vehicle.mode=VehicleMode('LAND')
vehicle.armed=False

print("EOS")

