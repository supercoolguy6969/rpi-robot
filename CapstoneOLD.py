########################################################################
# Filename    : Capstone.py
# Description : Multi-directional robot with 2 robotic arms
# auther      : Frederik Dupont
# created: 2023/01/02
# modified 2023/01/06
########################################################################
import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading

from RpiMotorLib import RpiMotorLib
import wiringpi as wiringpi




#MQTT Setup
# clientName = "Robot"
# serverAddress = "raspberrypi"
# mqttClient = mqtt.Client(clientName)

# # Flag to indicate subscribe confirmation hasn't been printed yet
# didPrintSubscribeMessage = False

# Setting Pin Mode
GPIO.setmode(GPIO.BCM)

#Controls Pins for Wheel's Stepper motor
motorDirFL = 27
motorStepFL = 22
motorDirFR = 23
motorStepFR = 18
motorDirBL = 13
motorStepBL = 19
motorDirBR = 16
motorStepBR = 12

# SENSORS
trigF = 4
echoF = 17
trigR = 25
echoR = 24
trigB = 26
echoB = 21
trigL = 5
echoL = 6

GPIO.setup(trigF, GPIO.OUT)   # set trig to OUTPUT mode
GPIO.setup(trigR, GPIO.OUT)   
GPIO.setup(trigB, GPIO.OUT)   
GPIO.setup(trigL, GPIO.OUT)   
GPIO.setup(echoF, GPIO.IN)    # set echo to INPUT mode
GPIO.setup(echoR, GPIO.IN)    
GPIO.setup(echoB, GPIO.IN)    
GPIO.setup(echoL, GPIO.IN)    

MAX_DISTANCE = 220          # define the maximum measuring distance, unit: cm
timeOut = MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance

# buzzer
buzzPin = 20
GPIO.setup(buzzPin, GPIO.OUT)

# I2C
SCL = 3
SDA = 2

# set the base number of ic1, this can be any number above (not including) 64
ic1_pin_base = 65
# pin number to code number:
# 1 = 65, 2 = 66, 3 = 67, 4 = 68, 5 = 69, 6 = 70, 7 = 71, 8 = 72, 9 = 73, 10 = 74, 11 = 75, 12 = 76, 13 = 77, 14 = 78, 15 = 79, 16 = 80
# define the i2c address of ic1, this is set by the jumpers on the HAT
ic1_i2c_addr = 0x24

# initiate the wiringpi library
wiringpi.wiringPiSetup()
# enable ic1 on the mcp23017 hat
wiringpi.mcp23017Setup(ic1_pin_base,ic1_i2c_addr)

# Arms via I2C
#Left Arm
servo1 = 65
servo2 = 66
servo3 = 67
servo4 = 68
servo5 = 69
servo6 = 70
# Right Arm
servo7 = 73
servo8 = 74
servo9 = 75
servo10 = 76
servo11 = 77
servo12 = 78


m = 1000

motorAll = RpiMotorLib.A4988Nema((motorDirFL,motorDirFR, motorDirBL, motorDirBR), (motorStepFL,motorStepFR, motorStepBL, motorStepBR), (-1,-1,-1), "DRV8825")
motorFRBL = RpiMotorLib.A4988Nema((motorDirFR,motorDirBL), (motorStepFR,motorStepBL), (-1,-1,-1), "DRV8825")
motorFLBR = RpiMotorLib.A4988Nema((motorDirFL,motorDirBR), (motorStepFL,motorStepBR), (-1,-1,-1), "DRV8825")
    

# Front LEFT, FRONT RIGHT, BACK LEFT, BACK RIGHT
def forward(step):
    motorAll.motor_go((False,True,False,True), "1/16",step,.00001,True,.05)  # for clockwise, we need False

def backward():   
    motorAll.motor_go((True,False,True,False), "1/16",10000,.00001,True,.05)

def left():
    motorAll.motor_go((True,True,False,False), "1/16",10000,.00001,True,.05)    
    
def right():
    motorAll.motor_go((False,False,True, True), "1/16",10000,.00001,True,.05)
    
def ccw():
    motorAll.motor_go((False,False,False,False), "1/16",1000,.00001,True,.05)
    
def cw():
    motorAll.motor_go((True,True,True,True), "1/16",1000,.00001,True,.05)
    
def diagFL():
    motorFRBL.motor_go((True,False), "1/16",1000,.00001,True,.05)
    
def diagFR():
    motorFLBR.motor_go((False,True), "1/16",1000,.00001,True,.05)
    
def diagBL():
    motorFLBR.motor_go((True,False), "1/16",1000,.00001,True,.05)
    
def diagBR():
    motorFRBL.motor_go((False,True), "1/16",1000,.00001,True,.05)

def destroy():
    GPIO.cleanup()                      # Release all GPIO

def stop():
    # motorAll.motor_stop()
    motorAll.motor_go((False,False,True,True), "1/16",0,.00005,True,.05)
    
def horn():
    GPIO.output(buzzPin, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(buzzPin, GPIO.LOW)
    
def pulseIn(pin,level,timeOut): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime
    
def getSonar(direction):     # get the measurement results of ultrasonic module,with unit: cm
    if direction == 'F':
        trigPin = trigF
        echoPin = echoF
    elif direction == 'L':
        trigPin = trigL
        echoPin = echoL
    elif direction == 'B':
        trigPin = trigB
        echoPin = echoB
    elif direction == 'R':
        trigPin = trigR
        echoPin = echoR
    else:
        print("invalid direction")
        
    GPIO.output(trigPin,GPIO.HIGH)      # make trigPin output 10us HIGH level 
    time.sleep(0.00001)     # 10us
    GPIO.output(trigPin,GPIO.LOW) # make trigPin output LOW level 
    pingTime = pulseIn(echoPin,GPIO.HIGH,timeOut)   # read plus time of echoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s 
    return distance


def sensor():
    while(True):
        distance1 = getSonar('L') # get distance
        print ("The distance Left is : %.2f cm"%(distance1))
        time.sleep(1)
        distance2 = getSonar('F') # get distance
        print ("The distance Front is : %.2f cm"%(distance2))
        time.sleep(1)
        distance3 = getSonar('R') # get distance
        print ("The distance Right is : %.2f cm"%(distance3))
        time.sleep(1)
        distance4 = getSonar('B') # get distance
        print ("The distance Back is : %.2f cm"%(distance4))
        time.sleep(1)
        
        
#For checking the connection status
def connectionStatus(client, userdata, flags, rc):
    print("subscribing")
    mqttClient.subscribe("robot/move")
    print("subscribed")

didPrintSubscribeMessage = False

# def connectionStatus(client, userdata, flags, rc):
    # global didPrintSubscribeMessage
    # if not didPrintSubscribeMessage:
        # didPrintSubscribeMessage = True
        # print("subscribing")
        # mqttClient.subscribe("pibot/move")
        # # print("subscribed")


# movement control
def messageDecoder(client, userdata, msg):
    
    #decodes the message 
    message = msg.payload.decode(encoding ='UTF-8')
    
    if message == "forward": 
        distance1 = getSonar('F') # get distance
        if distance1 >= 0:
                if tuning == True:
                    step=1000
                else:
                    step = 10000
            forward(step)
            print ("The Left distance is : %.2f cm"%(distance1))
            print("^^^ forward! ^^^") 
        else:
            stop()
         
    elif message == "backward":
        backward()
        print("\/ backward \/")
        
    elif message == "left":
        distance1 = getSonar('L') # get distance
        if distance1 >= 0:
            left()
            distance1 = getSonar('L') # get distance
            print ("The Left distance is : %.2f cm"%(distance1))
            print("<- left")
        else:
            stop()
        
    elif message == "right":
        right()
        print("-> right")
        
    elif message == "forwardleft":
        diagFL()
        print("\ Diagonal Front Left")
        
    elif message == "forwardright":
        diagFR()
        print("/ Diag Front Right")
        
    elif message == "backwardleft":
        diagBL()
        print("/ Diag Back Left")
        
    elif message == "backwardright":
        diagBR()
        print("\ Diag back Right")
        
    elif message == "clockwise":
        cw()
        print("O Clockwise")

    elif message == "ccw":
        ccw()
        print("O Counter Clockwise")
            
    elif message == "stop":
        stop()
        print("stopped")
        
    elif message == "stop":
        stop()
        print("stopped")
       
    else:
        print("?!? Unknown message?!?")



# For Frederik
# def messageDecoder(client, userdata, msg):
    
    # #decodes the message 
    # message = msg.payload.decode(encoding ='UTF-8')
    
    # if message == "forward": 
        # m=1
        # print("^^^ forward! ^^^") 
        
    # elif message == "backward":
        # m=1
        # print("\/ backward \/")
        
    # elif message == "left":
        # m = 3
        # print("<- left")
        
    # elif message == "right":
        # m = 4
        # print("-> right")
        
    # elif message == "forwardleft":
        # m = 5
        # print("\ Diagonal Front Left")
        
    # elif message == "forwardright":
        # m = 6
        # print("/ Diag Front Right")
        
    # elif message == "backwardleft":
        # m = 7
        # print("/ Diag Back Left")
        
    # elif message == "backwardright":
        # m = 8
        # print("\ Diag back Right")
        
    # elif message == "cw":
        # m = 9 
        # print("O Clockwise")

    # elif message == "ccw":
        # m = 10
        # print("ccw")
            
    # elif message == "stop":
        # m=100
        # print("stopped")
        
    # elif message == "horn":
        # m = 11
        # print("beep boop")
       
    # else:
        # loop(m)

    


#main function:
def loop(m):
    
    
    
    if m==1:
        if distance_sensor(F) > 20:
            forward()
    elif m==2:
        backward()
    elif m==3:
        left()
    elif m==4:
        right()
    elif m==5:
        diagFL()
    elif distance_sensor(F) > 20 and distance_sensor(L) > 20:
        m==6
        diagFR()
    elif m==7:
        diagBL()
    elif m==8:
        diagBR()
    elif m==9:
        cw()
    elif m==10:
        cww()
    elif m==100:
        stop()
    elif m==11:
        horn()
    else:
        print("please make a selection")

# if __name__ == "__main__" :
    # print ('Program is starting ... \n')
    # try:
        # m= messageDecoder(client, userdata, msg)
    
        # if m==1:
            # forward()
        # elif m==2:
            # backward()
        # elif m==3:
            # left()
        # elif m==4:
            # right()
        # elif m==5:
            # diagFL()
        # elif m==6:
            # diagFR()
        # elif m==7:
            # diagBL()
        # elif m==8:
            # diagBR()
        # elif m==9:
            # cw()
        # elif m==10:
            # cww()
        # elif m==100:
            # stop()
        # elif m==11:
            # horn()
        # else:
            # print("please make a selection")
        
    # except KeyboardInterrupt:   # Press ctrl-c to end the program.
        # destroy()

  
  
  
  

    




#MQTT Setup
clientName = "Raspberry Pi"
serverAddress = "raspberrypi"
mqttClient = mqtt.Client(clientName)

# Set up calling functions to mqttClient
mqttClient.on_connect = connectionStatus
mqttClient.on_message = messageDecoder


# Connect to the MQTT server & loop forever.
# CTRL-C will stop the program from running.
print("server address is:", serverAddress)
mqttClient.connect(serverAddress)
mqttClient.loop_forever()


    
# 
# if __name__ == '__main__':    # Program entrance
#     print ('Program is starting ... \n')
#     try:
#         horn()
#         forward()
#         time.sleep(1)
#         backward()
#         time.sleep(1)
#         left()
#         time.sleep(1)
#         right()
#         time.sleep(1)
#         diagFL()
#         time.sleep(1)
#         diagBR()
#         time.sleep(1)
#         diagFR()
#         time.sleep(1)
#         diagBL()
#         time.sleep(1)
#         cw()
#         time.sleep(1)
#         ccw()
        
    #except KeyboardInterrupt:   # Press ctrl-c to end the program.
       # destroy()

