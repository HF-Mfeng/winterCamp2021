import cv2
from dronekit import connect
from math import degrees, atan
from dronekitFly import speedControl, arm_and_takeoff, getHigh,yawControl,landing
import time
from PIDT import MFPID

vehicle = connect('/dev/ttyAMA0', baud = 921600, wait_ready = True)
pid = MFPID(0.005,0,0.001)

# 角度计算
def angleCount(upX,upY,downX,downY,wigth = 640, height = 480):
	if upX < 0 or upY < 0 or downX < 0 or downY < 0 or upX > wigth - 1 or downX > wigth - 1:
		return 0
	return degrees(atan((upX-downX)/(downY-upY))) # atan输出弧度，degrees讲弧度转为角度

def GroupByElement(lst, target = 0):
	if len(lst) == 0:
		return []
	i = 0 
	j = i + 1
	result = []
	temp = []
	length = len(lst)
	while  j <= length - 1:
		if lst[j] == lst[i]:
			temp.append(lst[i])
		else:
			temp.append(lst[i])
			result.append(temp)
			temp=[]
		i += 1
		j += 1
	temp.append(lst[j-1])
	result.append(temp)

	# 下面是自己写的
	lengOfResult = len(result)
	leftMaxIndex = -1
	maxLength = -1
	allIndex = 0
	x = 0
	while x < lengOfResult:
		if maxLength < len(result[x]) and result[x][0] == target:
			maxLength = len(result[x])
			leftMaxIndex = allIndex
		allIndex += len(result[x])
		x += 1
	return leftMaxIndex,maxLength,result

# 三点检测线
def three_point_line_fun(srcImage):
	gray = cv2.cvtColor(srcImage, cv2.COLOR_BGR2GRAY)
	ret1, threFrame = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
	kenerl = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	morFrame = cv2.morphologyEx(threFrame,1,kenerl)

	upline = 50
	midline = 240
	downline = 480 - upline

	# 检测上半部分
	centerX = -1
	left, length, lst = GroupByElement(morFrame[upline], 0)
	centerX = left + int(length/2)
	cv2.circle(srcImage,(centerX,upline),2,(0,0,255),2)
	cv2.circle(srcImage,(centerX,upline),32,(0,0,255),1)
	upCenterX = centerX
	upCenterY = upline

	# 检测中间
	centerX = -1
	left, length, lst = GroupByElement(morFrame[midline], 0)
	centerX = left + int(length/2)
	cv2.circle(srcImage,(centerX,midline),2,(0,0,255),2)
	cv2.circle(srcImage,(centerX,midline),32,(0,0,255),1)
	midCenterX = centerX
	midCenterY = midline

	# 检测下半部分
	centerX = -1
	left, length, lst = GroupByElement(morFrame[downline], 0)
	centerX = left + int(length/2)
	cv2.circle(srcImage,(centerX,downline),2,(0,0,255),2)
	cv2.circle(srcImage,(centerX,downline),32,(0,0,255),1)
	downCenterX = centerX
	downCenterY = downline

	return upCenterX, upCenterY, midCenterX, midCenterY, downCenterX, downCenterY

circle = 1

# 巡线前进
def forward(ux,uy,mx,my,dx,dy):
	up2downAngle = angleCount(ux,uy,dx,dy) # 上下角度
	mid2downAngle = angleCount(mx,my,dx,dy) # 中下角度
	if  abs( abs(up2downAngle) - abs(mid2downAngle) ) >= 15 : # 已经到圆了
		return circle
	centerAngle = (up2downAngle + mid2downAngle) / 2 # 中间的角度
	# 进行调整角度
	# yawControl(abs(centerAngle), (centerAngle/abs(centerAngle)),vehicle)
	
# 定点投放
def putdown(ux,uy,mx,my,dx,dy):
	print('putdown')
	
	time.sleep(2)


if __name__ == '__main__':
	cap = cv2.VideoCapture(0)
	draw = False
	line = 1
	print('take off')
	takeoffhigh = 0.4 # simple_takeoff 的高度
	arm_and_takeoff(takeoffhigh,vehicle)
	print('final hight:',getHigh(vehicle))
	pidtime = 0.06 # pid调节时间
	starttime = time.time() # 设置开始时间

	speed = 0.2 # 上升速度
	delay = 0.1 # 每次while循环延时时间
	allhigh = 1 # 需要总体上升高度
	times = ((allhigh-takeoffhigh)/speed ) * ( 1 / delay) + 30 # 计算次数，加了30次
	speedControl(0,0,speed,vehicle) 
	while times >= 0:
		times = times - 1
		time.sleep(delay)
		print('now hight:',getHigh(vehicle),'times:',times)
		if getHigh(vehicle) >= allhigh * 0.9 : # 高于总高度的0.9，就可以下降了
			break
	speedControl(0,0,0,vehicle)

	print('run line')
	while cv2.waitKey(1) != 27 :
		ret, src = cap.read()
		ux,uy,mx,my,dx,dy = three_point_line_fun(src)
		if time.time() - starttime > pidtime:
			speedControl(0.15,0,0,vehicle)
			if abs(320 - mx) > 20 :
				diff =  mx - 320 
				control = pid.MFPID_Output_Difference(diff)
				if abs(control) > 0.1:
					control = 0.1 * control / abs(control)
				speedControl(0.15,control,0,vehicle)
				print('pid speed:(',0.15,',',control,',',0,')')
			starttime = time.time()
		if line : # 在巡线中
			if forward(ux,uy,mx,my,dx,dy) == circle: # 遇到圆了
				line = 0
				break


	# 	# if line == 0 : # 已经到了圆了
	# 	# 	putdown(ux,uy,mx,my,dx,dy)
	# 	if draw:
	# 		cv2.line(src,(ux,uy),(mx,my),(255,0,0))
	# 		cv2.line(src,(dx,dy),(mx,my),(255,255,0))
	# 		cv2.line(src,(ux,uy),(dx,dy),(255,0,255))
	# 		cv2.imshow('src',src)


	print('release')
	speedControl(0,0,0.1,vehicle)
	time.sleep(3)
	speedControl(0,0,0,vehicle)
	time.sleep(2)
	landing(vehicle)
		
