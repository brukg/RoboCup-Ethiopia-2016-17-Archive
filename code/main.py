#packages for Threading and time
import time
import threading
import signal


#packages for Bluetooth
import serial

#Opencv and Numpy Packages
import cv2
import numpy as np
from collections import deque
import imutils



redLower = (158, 87, 75) # (160,100,82) # #
redUpper = (172, 255, 255)# (179,255,255) #""
blueLower = (49, 70, 74)
blueUpper = (117, 237, 255)
yellowLower = (18, 45, 151)
yellowUpper = (36, 244, 255)
redLower = (130, 113, 134) # (160,100,82) # #
redUpper = (179, 255, 255)
greenLower = (70, 132, 0)
greenUpper = (102, 255, 175)
blueLower = (102,138,76)
blueUpper =(166,205,161)
BallxyValue=''
goalxyValue=''
input_data=""
input_goal=""
readykick=0
walkingtoball=0
goalcounter=0

#intialize bluetooth communication
print("Communication started!")
port="/dev/rfcomm0" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
print("Bluetooth Connected !")
bluetooth.flushInput() #flush input buffer
bluetooth.flushOutput() #flush output buffer








'''
class NodeBluetooth(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		# The shutdown_flag is a threading.Event object that
		# indicates whether the thread should be terminated.
		self.shutdown_flag = threading.Event()
		# ... Other thread setup code here ...
	def run(self):
		print('[*]thread Bluetooth #%s started' % self.ident)
		global bluetooth
		global input_data
		while not self.shutdown_flag.is_set():
			# ... Job code here ...
			TmpData = bluetooth.inWaiting()
			TmpData = bluetooth.readline()
			if TmpData:
				input_data=TmpData.decode()[:-1].rstrip('\r')
				print("Message from Bluetooth  :"+ str(input_data))
			# ... Clean shutdown code here ...
		print('[+] thread Bluetooth stopped!')
'''
class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass

def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit

def Nodevision():
	global BallxyValue
	global baundaryxyValue
	global goalxyValue
	global bluetooth
	global redLower
	global redUpper
	# Register the signal handlerssignal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)
	print('[*] ------ Starting main program ----\n')
	# Start the job threads
	global camera
	camera = cv2.VideoCapture(0)
	# define the lower and upper boundaries of the "red"
	# ball in the HSV color space
	#bluetooth.write("")
	try:
		j1 = NodeController()
		#j2 = NodeBluetooth()
		j1.start()
		#j2.start()
		# Keep the main thread running, otherwise signals are ignored.

		while 1:

			(grabbed, ball) = camera.read()
			goal = ball.copy()
			baundary = ball.copy()

			# if we are viewing a video and we did not grab a frame,
			# then we have reached the end of the video
			if  not grabbed:
				break

			# resize the frame, blur it, and convert it to the HSV
			# color space
			ball = imutils.resize(ball, width=600)
			goal = imutils.resize(goal, width=600)
			baundary = imutils.resize(baundary, width=600)
			blurred = cv2.GaussianBlur(ball, (11, 11), 0)
			hsv = cv2.cvtColor(ball, cv2.COLOR_BGR2HSV)

			# construct a mask for the color "red", then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			maskball = cv2.inRange(hsv, redLower, redUpper)
			maskball = cv2.erode(maskball, None, iterations=2)
			maskball = cv2.dilate(maskball, None, iterations=2)
			maskgoal = cv2.inRange(hsv, greenLower, greenUpper)
			maskgoal = cv2.erode(maskgoal, None, iterations=2)
			maskgoal = cv2.dilate(maskgoal, None, iterations=2)
			maskbaundary = cv2.inRange(hsv, greenLower, greenUpper)
			maskbaundary = cv2.erode(maskbaundary, None, iterations=2)
			maskbaundary = cv2.dilate(maskbaundary, None, iterations=2)

			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cntsball = cv2.findContours(maskball.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)[-2]
			centerball = None
			cntsgoal = cv2.findContours(maskgoal.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)[-2]
			cntsbaundary = cv2.findContours(maskbaundary.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)[-2]
			centergoal = None
			# only proceed if at least one contour was found
			if len(cntsball) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				c = max(cntsball, key=cv2.contourArea)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				centerball = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				# only proceed if the radius meets a minimum size
				if radius > 18	:
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(ball, (int(x), int(y)), int(radius),
						(0, 255, 255), 2)
					BallxyValue=centerball
					cv2.circle(ball, centerball, 5, (0, 0, 255), -1)
				else:
					BallxyValue=''
			else:
				BallxyValue=''
			if len(cntsgoal) > 0 and  len(cntsbaundary) <=0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				cg = max(cntsgoal, key=cv2.contourArea)
				((xg, yg), radiusg) = cv2.minEnclosingCircle(cg)
				Mg= cv2.moments(cg)
				centergoal = (int(Mg["m10"] / Mg["m00"]), int(Mg["m01"] / Mg["m00"]))

				# only proceed if the radius meets a minimum size
				if radiusg > 10	:
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(goal, (int(xg), int(yg)), int(radiusg),
						(0, 255, 255), 2)
					goalxyValue=centergoal
					cv2.circle(goal, centergoal, 5, (0, 0, 255), -1)
				else:
					goalxyValue=''
			else:
				goalxyValue=''


			if len(cntsgoal) > 0 and len(cntsbaundary) > 0:



				cg = max(cntsgoal, key=cv2.contourArea)
				((xg, yg), radiusg) = cv2.minEnclosingCircle(cg)
				Mg = cv2.moments(cg)
				centergoal = (int(Mg["m10"] / Mg["m00"]), int(Mg["m01"] / Mg["m00"]))

				cb = max(cntsbaundary, key=cv2.contourArea)
				((xb, yb), radiusb) = cv2.minEnclosingCircle(cb)
				Mb = cv2.moments(cb)
				centerbaundary = (int(Mb["m10"] / Mb["m00"]), int(Mb["m01"] / Mb["m00"]))
				# only proceed if the radius meets a minimum size
				if radiusg > 10 and radiusb > 10:
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					if abs(centerbaundary[0]-centergoal[0])<20:
						cv2.circle(baundary, (int(xg), int(yg)), int(radiusg),
							(0, 255, 255), 2)
						goalxyValue=centergoal
						cv2.circle(baundary, centergoal, 5, (0, 0, 255), -1)

						cv2.circle(baundary, (int(xb), int(yb)), int(radiusb),
							(0, 255, 255), 2)
						baundaryxyValue=centerbaundary
						cv2.circle(baundary, centerbaundary, 5, (0, 0, 255), -1)
					elif abs(centerbaundary[0]-centergoal[0])<20:
						pass

				else:
					goalxyValue=''

			else:
				goalxyValue=''
				baundaryxyValue = ''
			# show the frame to our screen and increment the frame counter
			cv2.imshow("ball", ball)
			cv2.moveWindow("ball", 0, 0)
			#cv2.imshow("baundary", baundary)
			cv2.imshow("goal", goal)
			cv2.moveWindow("goal", 650, 0)
			#cv2.imshow("goal", goal)
			key = cv2.waitKey(1) & 0xFF
			# if the 'q' key is pressed, stop the loop
			if key == ord("q"):
				break
	except ServiceExit:
		#safly close Camera
		camera.release()
		cv2.destroyAllWindows()
		print('[+] thread Opencv Stopped!')
		#saftly close bluetooth
		bluetooth.close()
		# Terminate the running threads.
		# Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
		j1.shutdown_flag.set()
		# Wait for the threads to close...
		j1.join()
		print('\n  [+]Exiting Robot program \n')
class NodeController(threading.Thread):
	global BallxyValue
	def __init__(self):
		threading.Thread.__init__(self)
		# The shutdown_flag is a threading.Event object that
		# indicates whether the thread should be terminated.
		self.shutdown_flag = threading.Event()
		# ... Other thread setup code here ...
	def run(self):
		print('[*]thread Controller #%s started' % self.ident)
		global bluetooth
		global input_data
		global input_goal
		global BallxyValue
		global goalxyValue
		global readykick
		global walkingtoball
		global changepan
		global changetilt
		global goalcounter
		
		while not self.shutdown_flag.is_set():
			#print(BallxyValue)
			if BallxyValue and readykick==0:
				if (BallxyValue[0] > 375 or BallxyValue[0] < 225 or BallxyValue[1] < 150 or BallxyValue[1] > 300) :
					input_data=""
					print('adjusting to center')
					print BallxyValue
					#bluetooth.write('figure out')
					self.frame()
					readykick=0

				if BallxyValue[0] < 375 or BallxyValue[0] > 225 or BallxyValue[1] > 150 or BallxyValue[1] < 300:
					print "ball centered"
					#cv2.rectangle(frame,(375,300),(225,150),(0,255,0),3) #mark the Normal Bundary area
					bluetooth.flushOutput()
					print(input_data)
					bluetooth.flushInput()
					if input_data == "":
						bluetooth.write("GGG\n")
						time.sleep(0.3)

						#bluetooth.flushOutput()
						input_data = bluetooth.inWaiting()
						input_data = bluetooth.readline(input_data)
						input_data = input_data.rstrip()
						print ("incoming " + input_data)
					if input_data:
						result = input_data.split("&")
						print(result)
						if len(result)==3:
							headingDegrees=int(result[0])
							pospan=int(result[1])
							postilt=int(result[2])
							changepan =pospan-90
							changetilt= postilt-20
							print("delta pan " + str(changepan))
							print("delta tilt " + str(changetilt))
							if (changetilt) > 5 and abs(changepan) > 15:
								print("body tilt " )
								if changepan <-15:
									print('move right')
									bluetooth.write('z&'+str(changepan)+'\n')
									time.sleep(0.5)

								elif changepan>15:
									print('move left')
									bluetooth.write('Z&'+str(changepan)+'\n')
									time.sleep(0.5)


								while 1:
									res = bluetooth.inWaiting()
									if res:
										res = bluetooth.readline(res)
										res = res.rstrip()
										print(res)
										if res=="S":
											break
										elif res=="s":
											break
							if changetilt >= 5 and abs(changepan) <= 15:
								bluetooth.write("MMM\n")
								print("move to the ball")
								time.sleep(1)
							if changetilt <= 5 and abs(changepan) <= 30:
								readykick=1
								bluetooth.write("sss\n")
								print "ready to kick"
							if changetilt <= 5 and abs(changepan) > 30:
								print "Rotete Robot"
								if changepan < 0:
									bluetooth.write("R\n")
									time.sleep(1)
								else:
									bluetooth.write("L\n")
									time.sleep(1)
						
						else:
							input_data=""
					else:
						input_data=""
			elif readykick:
				input_data = ""
				if goalxyValue=='':
					if input_goal == '':
						bluetooth.write("J\n")
						time.sleep(0.2)
						bluetooth.flushOutput()
						input_goal = bluetooth.inWaiting()
						input_goal = bluetooth.readline(input_goal)
						input_goal = input_goal.rstrip()
						print ("get the direction of the robot")
						print  input_goal
					if input_goal:
						result = input_goal.split("&")
						print(result)
						if len(result)==3:
							headingDegrees=int(result[0])
							forward=int(result[1])
							backward=int(result[2])
							changefromforwardposition = abs(headingDegrees-forward)
							changefrombackwardpostion = abs(headingDegrees - backward)
							if abs(changefrombackwardpostion) <=60 or abs(changefrombackwardpostion) >=300 :
								print "turn 180 degrees and walk"
								bluetooth.write("o\n")
								while 1:
									res = bluetooth.inWaiting()
									if res:
										res = bluetooth.readline(res)
										res = res.rstrip()
										print(res)
										if res=="S":
											break
										elif res=="s":
											bluetooth.write("o\n")
										else:
											bluetooth.write("o\n")
								input_goal=""
								readykick=0
							elif abs(changefromforwardposition) <=45 or abs(changefromforwardposition) >=315 :
								bluetooth.write("N\n")
								print "searching goal"
								goalcounter+=1
								if goalcounter==30:
									bluetooth.write("W\n")
									goalcounter=0
								#if not BallxyValue:
									#readykick = 0
							else:
								bluetooth.write("k\n")
								while 1:
									res = bluetooth.inWaiting()
									if res:
										res = bluetooth.readline(res)
										res = res.rstrip()
										print(res)
										if res=="S":
											break
										elif res=="s":
											bluetooth.write("k\n")
										else:
											bluetooth.write("k\n")
								input_goal = ""
								readykick=0

						else:
							input_goal = ""

				if goalxyValue:
					if (goalxyValue[0] >= 350 or goalxyValue[0] <= 250 ) :
						input_data=""
						print('adjusting to goal')
						print BallxyValue
						#bluetooth.write('figure out')
						self.goalframe()

					if (goalxyValue[0] < 350 or goalxyValue[0] > 250):
						print "goal found get goal position from arduino"
						bluetooth.write("G\n")
						time.sleep(0.2)
						bluetooth.flushOutput()
						input_data = bluetooth.inWaiting()
						input_data = bluetooth.readline(input_data)
						input_data = input_data.rstrip()
						print ("get the angle of the goal")
						print  input_data
						if input_data:
							result = input_data.split("&")
							print(result)
							if len(result)==3:
								headingDegrees=int(result[0])
								pospan=int(result[1])
								changepan= headingDegrees-90
								if abs(changepan) <= 20:
									bluetooth.write("h\n")
									print "hit ball"
									time.sleep(1)
									bluetooth.write("M\n")
									print "kicking the ball towards the goal"
									readykick=0
								elif abs(changepan) > 20:
									if changepan < 0:
										bluetooth.write("R\n")
										print "step right "
									elif changepan > 0:
										bluetooth.write("L\n")
										print "step left"


				time.sleep(0.1)

			else:
				input_data = ""
				self.search()
			time.sleep(0.2)

	def search(self):
		global bluetooth
		bluetooth.write('n')
		print ('No Object Nothing detected!===>Searching ')
	def frame(self):
		global bluetooth
		global BallxyValue
		if BallxyValue[0] > 375:
			bluetooth.write('r\n')
		elif BallxyValue[0] < 225:
			bluetooth.write('l\n')
		elif BallxyValue[1] > 300:
			bluetooth.write('d\n')
		elif BallxyValue[1] < 150:
			bluetooth.write('u\n')
	def goalframe(self):
		global bluetooth
		global BallxyValue
		if goalxyValue[0] > 350:
			bluetooth.write('r\n')
		elif goalxyValue[0] < 250:
			bluetooth.write('l\n')

if __name__ == '__main__':

	response = raw_input("Please enter your command: ")
	bluetooth.write(str.encode(str(response)))
	time.sleep(0.5)
	Nodevision()
