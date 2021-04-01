from ImageTreat import Contour_Function, Line_Function
from dronekitFly import speedControl, arm_and_takeoff, getHigh,yawControl,landing
from dronekit import connect
from math import atan, degrees
from PWM import releaseThing
import time
import PIDT
import cv2


pid_time = 0.1 # pid调节时间

# 方位超调
upFix = 0
rightFix = 0

# 定义相机像素的宽，高，中心的宽，高
WIGTH = 640
HEIGH = 480
cx = WIGTH/2
cy = HEIGH/2
px1 = 5 # 定点中五个像素相当于没有差距

# 初始化无人机
vehicle = connect('/dev/ttyAMA0', baud = 921600, wait_ready = True)

speedcontrol = 0.3
# Times 表示调节1000次
def bottomCap_pointTakeoff(capture,pid,Times = 1000, maxSpeed = 2, draw = True):
	global pid_time,upFix,rightFix,WIGTH,HEIGH,cx,cy,vehicle
	global px1,speedcontrol
	# 为了底部摄像头的
	centerX = -1
	centerY = -1
	last_time = time.time()
	diffx = 0
	diffy = 0
	count = 0
	while Times > 1000:
		if cv2.waitKey(1) == 27:
			break
		if time.time() - last_time > pid_time:
			Times -= 1
			speedControl(0,0,0,vehicle)
			outcontrolx = 0
			outcontroly = 0
			if abs(diffx) > px1 : # 误差较大，需要较准
				outcontrolx = pid.MFPID_Output_Difference(diffx)
				if outcontrolx > maxSpeed:
					outcontrolx = maxSpeed
			if abs(diffy) > px1 :
				outcontroly = pid.MFPID_Output_Difference(diffy)
				if outcontroly > maxSpeed:
					outcontroly = maxSpeed
			speedControl(-outcontroly,outcontrolx,0,vehicle)
			last_time = time.time()
		ret, srcframe = capture.read()
		if ret:
			centerX, centerY = Contour_Function(srcframe)
			if centerX < 0 and centerY < 0: # 就是不在画面里面，就是需要调整了
				speedControl( upFix*0. , rightFix*0.1 ,0, vehicle)
			else:
				if centerX > cx + px1: # 在我右边
					rightFix = 1
				else centerX < cx - px1 and centerX > 0: # 左边
					rightFix = -1
				elif centerX - cx < px1 and centerX - cx > -px1: # 在中间
					rightFix = 0 # 在中间
				
				if centerY > cy + px1: # 在我下边
					upFix = -1
				else centerY < cy - px1 and centerY > 0: # 上边
					upFix = 1
				elif centerY - cy < px1 and centerY - cy > -px1: # 在中间
					upFix = 0 # 在中间
				
				diffx = centerX - cx # 大于0表示目标在右边,应去右
				diffy = centerY - cy # 大于0表示目标在下边,应去下
				if diffx < px1 and diffx > -px1 and diffy > -px1 and diffy < -px1:
					count += 1
				else:
					count = 0
				if count >= 10:
					break
				if draw:
					cv2.circle(out,(centerX,centerY),3,(255,0,0),3)
		if draw:
			cv2.imshow('src',srcframe)
	speedControl(0.2,0,0,vehicle)
	time.sleep(1)
	speedControl(0,0,0,vehicle) #之后进行角度调节


# 直线行驶
def bottomCap_runLine(capture,pid,Times = 1000, maxSpeed = 2, draw = True):
	global pid_time,upFix,rightFix,WIGTH,HEIGH,cx,cy,vehicle
	global px1,speedcontrol
	
def main():
	print('main')

if __name__ == '__main__':
	main()
