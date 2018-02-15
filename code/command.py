import serial
import time

print("Start")
#port="COM16" #This will be different for various devices and on windows it will probably be a COM port.
port="/dev/rfcomm0"
bluetooth=serial.Serial(port, 9600)#Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick
while True:
        response = raw_input("Please enter your command: ")
        bluetooth.write(str(response))
        '''time.sleep(8.2
        while response=="o" :
            res = bluetooth.inWaiting()
            response = bluetooth.readline(res)
            res = res.rstrip()
            print ("res "+ res)
            if res=="S":
                break
            elif res=="W":
                pass
            else:
                bluetooth.write("o\n")
        print "stopped"'''
        res = bluetooth.inWaiting()
        if res:
            res = bluetooth.readline(res)
            res = res.rstrip()
            print(res)
