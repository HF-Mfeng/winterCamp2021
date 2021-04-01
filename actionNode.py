#		参数说明：
#		PID：0.05 0.01 0.01 (还得调节)
#		图像坐标系:(320,240)                       	实际速度方位坐标系：
#		+------------------------------->x			y
#		|											^
#		|											|
#		|											|
#		|											|
#		|											|
#		|											|
#		|											|		
#		v  y										+------------------------------->x
from ImageTreat import Contour_Function, Line_Function
from dronekitFly import speedControl, arm_and_takeoff, getHigh,yawControl,landing
from dronekit import connect
from math import atan, degrees
from PWM import releaseThing
import time
import PIDT
import cv2
# 定义相机像素宽，高
WIGTH = 640
HEIGH = 480
# 定义中心宽，高
cx = WIGTH/2
cy = HEIGH/2

# 生成vehicle对象
vehicle = connect('/dev/ttyAMA0', baud = 921600, wait_ready = True)

# 计算角度的，中线左边为-，中线右边为+
def angleCount(upX,upY,downX,downY,wigth = 640, height = 480):
	if upX < 0 or upY < 0 or downX < 0 or downY < 0 or upX > wigth - 1 or downX > wigth - 1:
		return 0
	return degrees(atan((upX-downX)/(downY-upY))) # atan输出弧度，degrees讲弧度转为角度

# 进行角度修正，只需要用dronekit的库就行，不需要进行PID调节
def angleFix(capture,angle,Times = 500):
	global vehicle
	angle = 0
	while Times > 0:
		Times -= 1
		ret, srcFrame = capture.read()
		if ret:
			Line_up_X,Line_up_Y,Line_down_X,Line_down_Y = Line_Function(srcFrame, draw=False) # 直线检测
			if Line_up_X > 0 and Line_down_X > 0:
				angle = angleCount()
			else:
				speedControl(0.1,0,0)
				time.sleep(0.2)

	if angle > 7:
		yawControl(abs(angle),1,vehicle)
	elif angle < -7:
		yawControl(abs(angle),-1,vehicle)
	else:
		time.sleep(1)

# 定点起飞，使用直线检测法，以画面上为优先级最高来进行较准
# capture: 摄像头
# pid: pid调节
# hight: 起飞到多高
# sample_time: pid调节时间间隔
# px: 像素差多少认为是可以的
# maxspeed: 最大限制速度，防止冲的太快，损坏飞机
# Times: 调节次数
def pointTakeoff(capture,pid,hight = 1,sample_time = 0.1,px = 10,maxspeed = 2,Times = 1000):
	global cx
	global cy
	global vehicle
	arm_and_takeoff(hight,vehicle)
	time.sleep(2)
	while Times > 0:
		Times -= 1
		ret, srcFrame = capture.read()
		if ret:
			Line_up_X,Line_up_Y,Line_down_X,Line_down_Y = Line_Function(srcFrame, draw=False) # 直线检测
			print(Line_up_X,',',Line_up_Y,',',Line_down_X,',',Line_down_Y)
			if Line_up_X >= 0: # 上半部分检测到线
				diff = Line_up_X - cx # 测得差距，如果diff < 0 -->  left move | diff > 0 --> right move | diff = 0 break
				controlOut = pid.MFPID_Output_Difference(diff)
				if controlOut < maxspeed:  # 如果速度太大了，需要进行限制
					speedControl(0,controlOut,0,vehicle)
					time.sleep(sample_time)
					speedControl(0,0,0,vehicle)
				if Line_up_X - cx > -px or Line_up_X - cx < px: # 如果离目标像素差px个，说明是正常的
					break
			elif Line_down_X >= 0: # 上半部分没有检测到线，下半部分检测到了线
				diff = Line_down_X - cx # 测得差距，如果diff < 0 -->  left move | diff > 0 --> right move | diff = 0 break
				controlOut = pid.MFPID_Output_Difference(diff)
				if controlOut < maxspeed:  # 如果速度太大了，需要进行限制
					speedControl(0,controlOut,0,vehicle)
					time.sleep(sample_time)
					speedControl(0,0,0,vehicle)
				if Line_down_X - cx > -px or Line_down_X - cx < px: # 如果离目标像素差px个，说明是正常的
					break 
			else:
				speedControl(0.2,0,0,vehicle)
				time.sleep(0.2)

# 巡线行驶
# capture: 摄像头
# pid: pid调节
# sample_time: pid调节时间间隔 
# px: 
# maxspeed: 
# Times: 
# forwordSpeed: 
# angleMax: 
def runLine(capture,pid,sample_time = 0.1,px = 5,maxspeed = 2,Times = 10000,forwordSpeed = 0.5,angleMax = 15):
	global cx
	global cy
	global vehicle
	while Times > 0:
		Times -= 1
		ret, srcFrame = capture.read()
		if ret:
			Line_up_X,Line_up_Y,Line_down_X,Line_down_Y = Line_Function(srcFrame, draw=False) # 直线检测
			print(Line_up_X,',',Line_up_Y,',',Line_down_X,',',Line_down_Y)
			diff = (Line_up_X + Line_down_X) / 2 - cx # 取两者中间为基准
			controlOut = 0
			if diff > px or diff < -px: # 看是否需要PID调节
				controlOut = pid.MFPID_Output_Difference(diff)
			# 计算角度，如果差的太大，就是该定点投放了
			angleTemp = angleCount(Line_up_X,Line_up_Y,Line_down_X,Line_down_Y)
			if angleTemp > angleMax or angleTemp < -angleMax:
				speedControl(0,0,0,vehicle)
				break  # 该定点投放了
			if controlOut < maxspeed :  # 如果速度太大了，需要进行限制
				speedControl(forwordSpeed,controlOut,0,vehicle)
				time.sleep(sample_time)
				speedControl(forwordSpeed,controlOut,0,vehicle)

# 投放物品
def putDown(capture,pid,sample_time = 0.1, px = 5, maxspeed = 2, Times = 1000, speed = 0.1):
	global cx
	global cy
	global vehicle
	while Times > 0:
		Times -= 1
		ret, srcFrame = capture.read()
		if ret:
			Line_up_X,Line_up_Y,Line_down_X,Line_down_Y = Line_Function(srcFrame, draw=False) # 直线检测
			print(Line_up_X,',',Line_up_Y,',',Line_down_X,',',Line_down_Y)
			if Line_up_X >= 0:
				diff = abs(angleCount(Line_up_X,Line_up_Y,Line_down_X,Line_down_Y))
				speedControl(0,speed,0,vehicle)
				time.sleep(sample_time)
				speedControl(0,0,0,vehicle)
			else:
				speedControl(0,0,0,vehicle)
				break
	# 投放控制输出PWM
				

# 拍照扇叶
def takePhoto(capture, Times = 1000):
	global vehicle
	speedControl(0,0,0.1,vehicle) # 拍上面的照片
	time.sleep(0.8)
	while True:
		ret, srcFrame = capture.read()
		if ret:
			cv2.imwrite('/home/pi/up.png',srcFrame)
	speedControl(0,0,-0.1,vehicle)
	time.sleep(1.2)
	speedControl(0,0,0,vehicle)
	# 拍左边的照片
	speedControl(0,-0.1,0,vehicle)
	time.sleep(0.5)
	while True:
		ret, srcFrame = capture.read()
		if ret:
			cv2.imwrite('/home/pi/left.png',srcFrame)
	speedControl(0,0.1,0,vehicle)
	time.sleep(1)
	while True: # 拍下面的照片
		ret, srcFrame = capture.read()
		if ret:
			cv2.imwrite('/home/pi/right.png',srcFrame)
	speedControl(0,0,0,vehicle)
	# 降落

def main():
	global vehicle
	capture = cv2.VideoCapture(0)
	pid = PIDT.MFPID(0.05,0,0)
	print('takeoff')
	pointTakeoff(capture,pid,Times = 1000)
	releaseThing()
	# print('--------------------------------------')
	# print('runLine')
	# #runLine(capture,pid,Times = 10)
	# print('---------------------------------------')
	# print('putDown')
	# putDown(capture,pid,Times = 10)
	# print('end')
	landing(vehicle)
# 测试代码：
if __name__ == '__main__':
	main()
