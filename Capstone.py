########################################################################
# Filename    : Capstone.py
# Description : Multi-directional robot with 2 robotic arms
# author      : Frederik Dupont
# Co-author   : Anker
# created: 2023/01/02
# modified 2023/03/29
########################################################################
import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
from time import sleep
from threading import Thread
from _thread import interrupt_main
import sys
from RpiMotorLib import RpiMotorLib

global m, base1, shoulder1, elbow1, wristRot1, wristRaise1, grip1, leftArm, rightArm, bothArm, base2, shoulder2, elbow2, wristRot2, wristRaise2, grip2
global current


#MQTT Setup
clientName = "Raspberry Pi"
serverAddress = "raspberrypi"
mqttClient = mqtt.Client(clientName)


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

def arm_setup():
    # Create I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)

    # Arms I2C
    pca.frequency = 50
    # 12 bit resolution, 50% duty cycle
    for i in range(12):
        pca.channels[i].duty_cycle = 0x7FFF
    # creates 12 channel servos and initialize them
    kit = ServoKit(channels=12)
    for i in range(12):               ## might be the same as above !?
        kit.servo[i].set_pulse_width_range(1000, 2000)


def servo1(location,direction,current):  
    if direction == 'L':
        if current[location]>0:
            kit.servo[location].angle = current[location] - 10
    if direction == 'R':
        if current[location]<180:
            kit.servo[location].angle =  current[location] + 10

def servo2(location1,location2,direction,current1,current2):  
    if direction == 'L':
        if (current1[location1]>0 and current2[location2] >0):
            kit.servo[location1].angle =  current1[location1] - 10
            kit.servo[location2].angle = current2[location2] - 10
    if direction == 'R':
        if (current1[location1]<180 and current2[location2] <180):
            kit.servo[location1].angle =  current1[location1] + 10
            kit.servo[location2].angle =  current2[location2] + 10



motorAll = RpiMotorLib.A4988Nema((motorDirFL,motorDirFR, motorDirBL, motorDirBR), (motorStepFL,motorStepFR, motorStepBL, motorStepBR), (-1,-1,-1), "DRV8825")
motorFRBL = RpiMotorLib.A4988Nema((motorDirFR,motorDirBL), (motorStepFR,motorStepBL), (-1,-1,-1), "DRV8825")
motorFLBR = RpiMotorLib.A4988Nema((motorDirFL,motorDirBR), (motorStepFL,motorStepBR), (-1,-1,-1), "DRV8825")
    

# Front LEFT, FRONT RIGHT, BACK LEFT, BACK RIGHT
def forward():
    motorAll.motor_go((False,True,False,True), "1/16",100,.00001,True,.05)  # for clockwise, we need False

def backward():   
    motorAll.motor_go((True,False,True,False), "1/16",100,.00001,True,.05)

def left():
    motorAll.motor_go((True,True,False,False), "1/16",100,.00001,True,.05)    
    
def right():
    motorAll.motor_go((False,False,True, True), "1/16",100,.00001,True,.05)
    
def ccw():
    motorAll.motor_go((False,False,False,False), "1/16",100,.00001,True,.05)
    
def cw():
    motorAll.motor_go((True,True,True,True), "1/16",100,.00001,True,.05)
    
def diagFL():
    motorFRBL.motor_go((True,False), "1/16",100,.00001,True,.05)
    
def diagFR():
    motorFLBR.motor_go((False,True), "1/16",100,.00001,True,.05)
    
def diagBL():
    motorFLBR.motor_go((True,False), "1/16",100,.00001,True,.05)
    
def diagBR():
    motorFRBL.motor_go((False,True), "1/16",100,.00001,True,.05)

def destroy():
    GPIO.cleanup()                      # Release all GPIO

def stop():
    motorAll.motor_stop()
    #motorAll.motor_go((False,False,True,True), "1/16",0,.00005,True,.05)
    
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

#For checking the connection status
def connectionStatus(client, userdata, flags, rc):
    print("subscribing")
    mqttClient.subscribe("robot/move")
    print("subscribed")



def thread_sensor1(direction): 
    while True:
        distance1 = getSonar(direction)
        if distance1 < 20:
            raise RpiMotorLib.StopMotorInterrupt   

def thread_sensor2(direction1, direction2): 
    while True:
        distance1 = getSonar(direction1)
        distance2 = getSonar(direction2)
        if (distance1 < 20|distance2 <20):
            raise RpiMotorLib.StopMotorInterrupt   

# class StopMotorInterrupt(Exception):
    # """ Stop the motor """
    # pass

#movement control
def messageDecoder(client, userdata, msg):
    try:
        message = msg.payload.decode(encoding ='UTF-8')
        
        if message == "forward": 
            thread = Thread(target=thread_sensor1, args= 'F')
            thread.setDaemon(True)
            thread.start()
            forward()
            print("^^^ forward! ^^^") 
            thread.join()
             
        elif message == "backward":
            backward()
            print("\/ backward \/")
            
        elif message == "left":
            distance1 = getSonar('L') # get distance
            if distance1 > 20:
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
            raise StopMotorInterrupt
            
        elif message == "buzz":
            horn()
            print("buzz")
            
        elif message == 'LeftArm':
            leftArm = True
            rightArm = False
            bothArm = False
        elif message == 'rightArm':
            leftArm = False 
            rightArm = True 
            bothArm = False 
        elif message == 'bothArm':
            leftArm = False 
            rightArm = False 
            bothArm = True 
        elif message == 'base': 
            if leftArm == True:
                base1 = True
                shoulder1 = False
                elbow1 = False
                wristRot1 = False
                wristRaise1 = False
                grip1 = False
                base2 = False
                shoulder2 = False
                elbow2 = False
                wristRot2 = False
                wristRaise2 = False
                grip2 = False
            elif rightArm == True:
                base1 = False
                shoulder1 = False
                elbow1 = False
                wristRot1 = False
                wristRaise1 = False
                grip1 = False
                base2 = True
                shoulder2 = False
                elbow2 = False
                wristRot2 = False
                wristRaise2 = False
                grip2 = False
            elif bothArm == True:
                base1 = True
                shoulder1 = False
                elbow1 = False
                wristRot1 = False
                wristRaise1 = False
                grip1 = False
                base2 = True
                shoulder2 = False
                elbow2 = False
                wristRot2 = False
                wristRaise2 = False
                grip2 = False
        elif message == 'UP':
            if base1 == True and base2 == False:
                servo1(0,'R', current[0])
            elif base1 == False and base2 == True:
                servo1(6,'R', current[6])
            elif base1 == True and base2 == True:
                servo2(0,6,'R', current[0], current[6])
        elif message == 'DOWN':
            if base1 == True and base2 == False:
                servo1(0,'L', current[0])
            elif base1 == False and base2 == True:
                servo1(6,'L', current[6])
            elif base1 == True and base2 == True:
                servo2(0,6,'L', current[0], current[6])
        
        
    except RpiMotorLib. StopMotorInterrupt:
        RpiMotorLib.StopMotorInterrupt(Exception)
        
    ### For the arms 
    ## elif message == "elbow"
    ## elif message == "shoulder"
    ## elif message == "base"
    ## elif message == "grip"
    ## elif message == "wrist_rotate"
    ## elif message == "wrist_vertical"
    
    
    # else:
        # print("?!? Unknown message?!?")


        
    

if __name__ == "__main__" :
    print ('Program is starting ... \n')
    try:
        # global current
        # for i in range(12):
            # current[i]=0
        # start the new thread
        # # thread = Thread(target=task)
        # # thread.start()
        
        # Set up calling functions to mqttClient
        mqttClient.on_connect = connectionStatus
        mqttClient.on_message = messageDecoder


        # Connect to the MQTT server & loop forever.
        # CTRL-C will stop the program from running.
        print("server address is:", serverAddress)
        mqttClient.connect(serverAddress)
        mqttClient.loop_forever()

    except RpiMotorLib.StopMotorInterrupt:
        print("Stop Motor Interrupt")
    except KeyboardInterrupt:   # Press ctrl-c to end the program.
        destroy()

  
