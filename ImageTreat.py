import cv2
import time



###################算法和调参#######################
### 分组算法
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
### PID算法

### 配置信息
camera = 1
contour_kenerl1Vaule = 158
contour_kenerl2Vaule = 1
contour_thresholdValue = 127
### 调参
def update_contour_kenerl1Vaule(value):
	global contour_kenerl1Vaule
	contour_kenerl1Vaule = value
def update_contour_kenerl2Vaule(value):
	global contour_kenerl2Vaule
	contour_kenerl2Vaule = value
def update_contour_thresholdValue(value):
	global contour_thresholdValue
	contour_thresholdValue = value
def createContourOptionTrackBar(contour_kenerl1Vaule_need = True, contour_kenerl2Vaule_need = True, contour_thresholdValue_need = True):
	global contour_kenerl1Vaule
	global contour_kenerl2Vaule 
	global contour_thresholdValue
	windowName = 'ContourOption'
	cv2.namedWindow(windowName)
	if contour_kenerl1Vaule_need:
		cv2.createTrackbar('kenerl1Value:',windowName,contour_kenerl1Vaule,640,update_contour_kenerl1Vaule)
	if contour_kenerl2Vaule_need:
		cv2.createTrackbar('kenerl2Value:',windowName,contour_kenerl2Vaule,320,update_contour_kenerl2Vaule)
	if contour_thresholdValue_need:
		cv2.createTrackbar('thresholdValue:',windowName,contour_thresholdValue,255,update_contour_thresholdValue)
	# cv2.createTrackbar(':',windowName, ,639,update_)

Line_thresholdValue = 89 # 起飞以前为第一
Line_morphologyMode = 1
Line_KenerlValue1 = 3
Line_KenerlValue2 = 3
Line_upLine = 20
Line_downLine = 480-20
def  update_Line_downLine(value):
	global Line_downLine
	Line_downLine=value
def  update_Line_thresholdValue(value):
	global Line_thresholdValue
	Line_thresholdValue=value
def  update_Line_morphologyMode(value):
	global Line_morphologyMode
	Line_morphologyMode=value
def  update_Line_KenerlValue1(value):
	global Line_KenerlValue1
	Line_KenerlValue1=value
def  update_Line_KenerlValue2(value):
	global Line_KenerlValue2
	Line_KenerlValue2=value
def  update_Line_upLine(value):
	global Line_upLine
	Line_upLine=value
def createLineOptionTrackBar(Line_thresholdValue_need = True,Line_morphologyMode_need = False, Line_KenerlValue1_need = True, Line_KenerlValue2_need = True, Line_upLine_need = False, Line_downLine_need = False):
	global Line_thresholdValue 
	global Line_morphologyMode 
	global Line_KenerlValue1 
	global Line_KenerlValue2 
	global Line_upLine 
	global Line_downLine 
	windowName = 'LineOption'
	cv2.namedWindow(windowName)
	if  Line_thresholdValue_need:
		cv2.createTrackbar('thresholdValue:',windowName,Line_thresholdValue ,255,update_Line_thresholdValue)
	if  Line_morphologyMode_need:
		cv2.createTrackbar('MorphMode:',windowName,Line_morphologyMode ,7,update_Line_morphologyMode)
	if  Line_KenerlValue1_need:
		cv2.createTrackbar('kenerValue1:',windowName,Line_KenerlValue1 ,10,update_Line_KenerlValue1)
	if  Line_KenerlValue2_need:
		cv2.createTrackbar('kenerValue2:',windowName, Line_KenerlValue2,10,update_Line_KenerlValue2)
	if  Line_upLine_need:
		cv2.createTrackbar('upLine:',windowName, Line_upLine,240,update_Line_upLine)
	if  Line_downLine_need:
		cv2.createTrackbar('downLine:',windowName, Line_downLine,480,update_Line_downLine)

def createPIDTOptionTrackBar(MK_P,MP_I,MP_D,MP_Time):
	windowName = 'PIDTOption'
###################算法和调参上面#######################

### 检测轮廓
def Contour_Function(srcImage,kenerl1Value = 158,kenerl2Value = 1,thresholdValue = 127,draw = False,allContours = False):
	grayImg = cv2.cvtColor(srcImage,cv2.COLOR_BGR2GRAY)
	ret1, thresholdFrame = cv2.threshold(grayImg,thresholdValue,255,cv2.THRESH_BINARY)
	kenerl = cv2.getStructuringElement(cv2.MORPH_RECT,(kenerl1Value,kenerl2Value))
	morFrame = cv2.morphologyEx(thresholdFrame,3,kenerl)
	morFrame = ~morFrame
	if draw:
		cv2.imshow('mor',morFrame)
		cv2.imshow('~mor',~morFrame)
	contours, hi = cv2.findContours(morFrame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	centerX = -1
	centerY = -1
	# 下面是寻找最大轮廓
	if not allContours:
		max = 0
		maxindex = -1
		for index, value in enumerate(contours):
			temp = cv2.contourArea(value)
			if max <= temp:
				max = temp
				maxindex = index
		x = -1
		y = -1
		w = -1
		h = -1
		if maxindex != -1:
			x, y, w, h = cv2.boundingRect(contours[maxindex])
		if draw:
			cv2.drawContours(srcImage,contours,maxindex,(255,0,0),3)
			cv2.circle(srcImage,((int)(x+w/2),(int)(y+h/2)),2,(0,0,255),2)
			print('index : ' , maxindex , ', center = (' , (x+w/2), ',' , (y+h/2), ')')
		centerX = (int)(x+w/2)
		centerY = (int)(y+h/2)
	else:
		if draw:
			print('--------------------------------------------')
			print('contours.len = ', len(contours) )
		for index, value in enumerate(contours):
			x, y, w, h = cv2.boundingRect(value)
			if draw:
				cv2.circle(out,((int)(x+w/2),(int)(y+h/2)),2,(255,0,0),2)
				cv2.drawContours(srcImage,contours,index,(255,0,0),3)
				print('index : ' , index , ', center = (' , (x+w/2), ',' , (y+h/2), ')')
	return centerX, centerY

# 三点检测线
def three_point_line_fun(srcImage):
	gray = cv2.cvtColor(srcImage, cv2.COLOR_BGR2GRAY)
	ret1, threFrame = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
	kenerl = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	morFrame = cv2.morphologyEx(threFrame,1,kenerl)

	upline = 120
	midline = upline * 2
	downline = upline * 3

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
	
# 检测线
def Line_Function(srcImage,thresholdValue = 127,MorphologyMode = 1,kenerlVaule1 = 3,kenerlVaule2 = 3,upCheckLine = 20,downCheckLine = 480-20,draw = True):
	grayFrame = cv2.cvtColor(srcImage,cv2.COLOR_BGR2GRAY)
	ret1, thresholdFrame = cv2.threshold(grayFrame,thresholdValue,255,cv2.THRESH_BINARY)
	kenerl = cv2.getStructuringElement(cv2.MORPH_RECT,(kenerlVaule1,kenerlVaule2))
	morFrame = cv2.morphologyEx(thresholdFrame,MorphologyMode,kenerl)
	if draw:
		cv2.imshow('morFrame',morFrame)
		back = ~morFrame
		cv2.imshow('~morFrame',back)
	# 检测上半部分
	centerX = -1
	left, length, lst = GroupByElement(morFrame[upCheckLine], 0)
	centerX = left + int(length/2)
	if centerX != -1 and draw:
		cv2.circle(srcImage,(centerX,upCheckLine),2,(0,0,255),2)
		cv2.circle(srcImage,(centerX,upCheckLine),32,(0,0,255),1)
	upCenterX = centerX
	upCenterY = upCheckLine
	# 检测下半部分
	centerX = -1
	left, length, lst = GroupByElement(morFrame[downCheckLine], 0)
	centerX = left + int(length/2)
	if centerX != -1 and draw:
		cv2.circle(srcImage,(centerX,downCheckLine),2,(0,0,255),2)
		cv2.circle(srcImage,(centerX,downCheckLine),32,(0,0,255),1)
	downCenterX = centerX
	downCenterY = downCheckLine
	return upCenterX,upCenterY,downCenterX,downCenterY

def main(TrackBar = True, timeShow = False, draw = True):
	global contour_kenerl1Vaule
	global contour_kenerl2Vaule 
	global contour_thresholdValue
	global camera
	global Line_thresholdValue 
	global Line_morphologyMode 
	global Line_KenerlValue1 
	global Line_KenerlValue2 
	global Line_upLine 
	global Line_downLine
	if TrackBar:
		#createLineOptionTrackBar()
		createContourOptionTrackBar()
	capture = cv2.VideoCapture(camera)

	ContourCenterX = -1
	ContourCenterY = -1

	Line_up_X = -1
	Line_up_Y = -1
	Line_down_X = -1
	Line_down_Y = -1

	while cv2.waitKey(1) != 27:
		if timeShow:
			start = time.time()
		ret, srcframe = capture.read()
		if ret: #图像处理
			# 直线检测
			# Line_up_X,Line_up_Y,Line_down_X,Line_down_Y = Line_Function(srcframe,Line_thresholdValue,Line_morphologyMode,Line_KenerlValue1,Line_KenerlValue2,Line_upLine,Line_downLine,True)
			# print(Line_up_X,',',Line_up_Y,',',Line_down_X,',',Line_down_Y)
			# 轮廓检测
			ContourCenterX,ContourCenterY = Contour_Function(srcframe,contour_kenerl1Vaule,contour_kenerl2Vaule, contour_thresholdValue,True,False)
			print(ContourCenterX,',',ContourCenterY)
			if draw:
				cv2.imshow('src',srcframe)
		if timeShow:
			end = time.time()
			print('fps = ' , 1/(end - start))
	cv2.destroyAllWindows()
if __name__ == '__main__':
	main()
