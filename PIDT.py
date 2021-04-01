class MFPID:
	def __init__(self, P, I, D, SetPoint = 0 ):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.setPoint = SetPoint
		self.last_err = 0.0
		self.all_err = 0.0

	def setPID(self,KP,KI,KD):
		self.Kp = KP
		self.Ki = KI
		self.Kd = KD
# 输入此时的值
	def MFPID_Output_Value(self, value):
		err = self.setPoint - value
		self.MFPID_Output_Difference(err)

# 输入此时的差距
#Difference_value就是此时的差值，有正有负，目标-本身
	def MFPID_Output_Difference(self,Difference_value):
		#比例
		PItem = self.Kp * Difference_value
		#积分
		IItem = self.Ki * self.all_err
		#微分
		DItem = self.Kd * (Difference_value - self.last_err)
		outValue = PItem + IItem + DItem
		self.last_err = Difference_value
		self.all_err += self.last_err
		return outValue
		

def main():
	pid = MFPID(0.1,0.1,0.1)
	Point = 100
	NowPoint = 0
	while True:
		temp = pid.MFPID_Output_Difference(Point-NowPoint)
		NowPoint += temp
		print('temp = ',temp,' NowPoint = ', NowPoint)
		if( (NowPoint - Point < 0.01) and (NowPoint - Point > -0.01)):
			break
if __name__ == '__main__':
	main()
