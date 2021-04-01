from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# set EKF Origin Here
def set_Home(vehicle):
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        0,0,0,
        
       # 0,0,1)
        322486826,1186472457,12190)
    ret = vehicle.send_mavlink(msg)
    print(ret)
   # vehicle.flush()

# direction为1就是顺时针，-1就是逆时针
# angle是角度
# speed是旋转速度
def yawControl(angle,direction,vehicle,speed = 45):
        if speed>0 and speed<=1080:
                while vehicle.parameters['ATC_RATE_Y_MAX']!=speed:
                        vehicle.parameters['ATC_RATE_Y_MAX']=speed
                        time.sleep(1)
        msg = vehicle.message_factory.command_long_encode(
                0,0,    #target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,#command
                0,#confirmation
                angle,  #param 1,yaw in degrees
                0,#param 2,yaw in speed deg/s此处速度参数不起作用
                direction,#方向1为顺时针，-1逆时针
                1,#选择相对角度
                0,0,0)   #param 5~7 不使用
        #发送指令
        vehicle.send_mavlink(msg)

# 速度控制
# 前右上为正
def speedControl(forward,right,up,vehicle):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0,       # time_boot_ms (not used)
                    0, 0,    # target system, target component
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,   # frame
                    0b0000111111000111, # type_mask (only speeds enabled)
                    0, 0, 0, # x, y, z positions (not used)
                    forward, right, -up, # x, y, z velocity in m/s
                    0, 0, 0, # x, y, z acceleration (not used)
                    0, 0)    # yaw, yaw_rate (not used) 
    #发送指令
        ret = vehicle.send_mavlink(msg)
        #print('ret is ' , ret)

# 起飞到一定高度
gMode = 'GUIDED'
def arm_and_takeoff(hight,vehicle):
        global gMode
        vehicle.mode = VehicleMode(gMode)
        vehicle.armed=True
        time.sleep(1)
        vehicle.armed = False
        time.sleep(5)
        print('change mode')
        while vehicle.mode.name != gMode:
                vehicle.mode = VehicleMode(gMode)
                time.sleep(1)
        #print('will arm the vehicle')
        #time.sleep(2)
        print('will arm the vehicle')
        time.sleep(2)
        while vehicle.armed != True:
                vehicle.armed = True
                time.sleep(2)
        time.sleep(4)
        vehicle.simple_takeoff(hight)
        time.sleep(2)

        while True:
                if vehicle.rangefinder.distance >= 0.9 * hight :
                        speedControl(0,0,0,vehicle)
                        break
                else:
                        print('vehicle distance:',vehicle.rangefinder.distance)
                        time.sleep(2)
        time.sleep(1)
        # 降落

def landing(vehicle,high = 1):
        # speed = 0.1
        # speedControl(0,0,-speed,vehicle)
        # time.sleep(0.65/speed)
        speed = 0.1 # 下降速度
        speedControl(0,0,-speed,vehicle)
        delay = 0.1 # 每次while循环延时
        minhigh = 0.4 # 下降到最低速度就停止
        times = ( high - minhigh) / speed + 20
        while times >= 0:
                time.sleep(delay)
                if vehicle.rangefinder.distance < minhigh:
                        break
        speedControl(0,0,0,vehicle)
        vehicle.mode = VehicleMode("LAND")
        time.sleep(2)
        vehicle.armed = False
        vehicle.close()

# 获取当前高度
def getHigh(vehicle):
	return vehicle.rangefinder.distance

def main():
	print('test')

if __name__ == '__main__':
	main()
