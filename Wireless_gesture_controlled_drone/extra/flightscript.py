import socket
#we did it :)
from dronekit import connect,VehicleMode, LocationGlobalRelative,APIException
from pymavlink import mavutil
import time
import socket
import math
import argparse

HOST=""
PORT=65432
maxheight=4
minheight=0.5
height=0.5
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

def is_socket_closed(sock: socket.socket):
    try:
        # this will try to read bytes without blocking and also without removing them from buffer (peek only)
        data = sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
        if len(data) == 0:
            return True
    except BlockingIOError:
        return False  # socket is open and reading from it would block
    except ConnectionResetError:
        return True  # socket was closed for some other reason
    except Exception as e:
        return False
    return False

# place laptop in north in starting
up=-1
down=1
forward=0.5  # north
back=-0.5     #south
right=0.5      #east
left=-0.5     #west
respond=True






last_command='-1'
miss=5
landing_miss=15
cdown=landing_miss
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
    s.bind((HOST,PORT))
    conn,addr=s.accept()
    last_valid_gesture="vertical"
    vehicle=connectMyCopter()
    arm()
    vehicle.simple_takeoff(2)
    time.sleep(10)
    with conn:
        while True:
            if is_socket_closed(s)==False:
                unarm()
            data=conn.recv(1024)
            data=data.decode("utf-8")
            if last_command==data:
                miss-=1
            else:
                miss=5
            if last_command==data and data=='3':
                landing_miss-=1
            else:
                landing_miss=15
            if data=='esc':
                vehicle.mode=VehicleMode('LAND')
                vehicle.armed=False
                break
            elif data=='space':
                vehicle.simple_takeoff(2)

            elif data=='-1' and respond==True :
                #No input
                print("No hand detected drone in still mode")
                continue
            elif data=='2' and vehicle.armed==True and respond==True and miss<=0:
                #up
                print("Going UP")
                send_ned_velocity(0, 0, up)

            elif data=='4' and vehicle.armed==True and respond==True and miss<=0:
                #down
                print("Going Down")
                send_ned_velocity(0, 0, down)

            elif data=='0' and vehicle.armed==True and respond==True and miss<=0:
                #Forward
                print("Going Forward")
                send_ned_velocity(forward,0,0)

            elif data=='5' and vehicle.armed==True and respond==True and miss<=0:
                #back
                print("Going Back")
                send_ned_velocity(back,0,0)

            elif data=='6' and vehicle.armed==True and respond==True and miss<=0:
                #left
                print("Going Left")
                send_ned_velocity(0,left,0)

            elif data=='7' and vehicle.armed==True and respond==True and miss<=0:
                #right
                print("Going Right")
                send_ned_velocity(0,right,0)

            elif data=='3':
                #land
                landing_miss-=1
                print("landing begin in ",cdown-landing_miss)
                if landing_miss<0:
                    respond=False
                    print("Landing....")
                    unarm()
                    print("Unarmed")

            elif data=='1'  and vehicle.armed==True and respond==True:
                #stop
                print("Stop")
                send_ned_velocity(0,0,0)
            last_command=data
        unarm()
    unarm()

unarm()


print("END OF SCRIPT")

            