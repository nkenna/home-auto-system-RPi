#!/usr/bin/env python
import RPi.GPIO as GPIO
import sys
import serial
from time import sleep, strftime, time
import Adafruit_DHT
import socket
from pubnub.pubnub import PubNub
from pubnub.callbacks import SubscribeCallback
from pubnub.enums import PNOperationType, PNStatusCategory, PNReconnectionPolicy
from pubnub.pnconfiguration import PNConfiguration
from RPIO import PWM
from numpy import interp
import multiprocessing

import random



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

temp_sensormodel = Adafruit_DHT.AM2302  # setup am2302 sensor
temp_sensorpin = 4  # setup am2302 sensor pin

pinGarageLight = 5
pinAlarm = 17
pinRelay_fan = 22
pinBedroomLight = 26
pinSittingRoomLight = 16
pinPIR_garage = 23
pinPIR_bedroom = 25
pinPIR_sittingroom = 24
pinNetwork = 6
pinSecurityLight = 27
pinFrontDoorServo = 18
pingarageDoorServo = 12
pinCServo = 13


GPIO.setup(pinGarageLight , GPIO.OUT)
GPIO.setup(pinAlarm , GPIO.OUT)
GPIO.setup(pinRelay_fan, GPIO.OUT)
GPIO.setup(pinBedroomLight, GPIO.OUT)
GPIO.setup(pinSittingRoomLight, GPIO.OUT)
GPIO.setup(pinPIR_garage, GPIO.IN)
GPIO.setup(pinPIR_bedroom, GPIO.IN)
GPIO.setup(pinPIR_sittingroom, GPIO.IN)
GPIO.setup(pinNetwork, GPIO.OUT)
GPIO.setup(pinSecurityLight , GPIO.OUT)

GPIO.output(pinRelay_fan, GPIO.HIGH)



CServo = PWM.Servo()
sleep(1)
CServo.set_servo(pinCServo, 1500)


p = GPIO.PWM(pinNetwork, 50)     # set Frequece to 1KHz
p.start(0)

bedroom = GPIO.PWM(pinBedroomLight, 50)     # set Frequece to 1KHz
bedroom.start(0)

sittingroom = GPIO.PWM(pinSittingRoomLight, 50)     # set Frequece to 1KHz
sittingroom.start(0)

securitylite = GPIO.PWM(pinSecurityLight, 50)     # set Frequece to 1KHz
securitylite.start(0)

garagelite = GPIO.PWM(pinGarageLight, 50)
garagelite.start(0)

fDoorServo = PWM.Servo()
garageDoorServo = PWM.Servo()

garageDoorServo.set_servo(pingarageDoorServo, 800)
sleep(1)
garageDoorServo.stop_servo(pingarageDoorServo)
fDoorServo.set_servo(pinFrontDoorServo, 800)
sleep(1)
fDoorServo.stop_servo(pinFrontDoorServo)

# pubnub channels
LIGHTS = 'LIGHTS'
DOORS = 'DOORS'
ELECTRICAL = 'ELECTRICAL'
CAMERA = 'CAMERA'
SENSORS = 'SENSORS'
SENSORS_TEMP = 'SENSORS_TEMP'
SENSORS_HUMID = 'SENSORS_HUMID'
FEEDBACKS = 'FEEDBACKS'
MODE =  'MODE'
PHOTO = 'PHOTO'
GARAGE_DOOR = 'GARAGE_DOOR'
FRONT_DOOR = 'FRONT_DOOR'
FAN = 'FAN'
BEDROOM_LIGHT = 'BEDROOM_LIGHT'
SITTINGROOM_LIGHT = 'SITTINGROOM_LIGHT'
GARAGE_LIGHT = 'GARAGE_LIGHT'
SECURITY_LIGHT = 'SECURITY_LIGHT'


# pubnub payloads
# get sensors' data
sensors_Data = 'get data'

# operating lights
securityLight_ON = 'SECURITY LIGHT ON'
securityLight_OFF = 'SECURITY LIGHT OFF'

garageLight_ON = 'GARAGE LIGHT ON'
garageLight_OFF = 'GARAGE LIGHT OFF'

sittingRoomLight_ON = 'SITTING ROOM LIGHT ON'
sittingRoomLight_OFF = 'SITTING ROOM LIGHT OFF'

bedroomLight_ON = 'BEDROOM LIGHT ON'
bedroomLight_OFF = 'BEDROOM LIGHT OFF'

# operating doors
garageDoor_OPEN = 'GARAGE DOOR OPEN'
garageDoor_CLOSE = 'GARAGE DOOR CLOSE'

frontDoor_OPEN = 'FRONT DOOR OPEN'
frontDoor_CLOSE = 'FRONT DOOR CLOSE'

# operating electrical appliances
fan_ON = 'FAN ON'
fan_OFF = 'FAN OFF'

light_ON = 'LIGHT ON'
light_OFF = 'LIGHT OFF'

# operating camera
camera_Right = 'RIGHT'  # right is to sitting room
camera_Left = 'LEFT'  # left is to bedroom
camera_pic = 'PIC'

# changing mode
changeMode_AUTO = 'AUTO'
changeMode_MANUAL = 'MANUAL'

# detecting motion
MOTION_PIR1 = 'BEDROOM MOTION'
MOTION_PIR2 = 'GARAGE MOTION'
MOTION_PIR3 = 'SITTINGROOM MOTION'




# initialize pubnub
pnconfig = PNConfiguration()
pnconfig.subscribe_key = '*******************'
pnconfig.publish_key = '**********************'



pubnub = PubNub(pnconfig)

def garageMotion(pinPIR_garage):
    if sm.securityStatus:
        if GPIO.input(pinPIR_garage):
            print('pinPIR_garage motion detected')
            GPIO.output(pinAlarm , GPIO.HIGH)
            GPIO.output(pinGarageLight, GPIO.HIGH)
            sleep(1)
            CServo.set_servo(pinCServo, 1500)
            pubnub.publish().channel(SENSORS).message(MOTION_PIR2).async(my_publish_callback)
        else:
            print('pinPIR_garage detection ends')
            GPIO.output(pinAlarm, GPIO.LOW)
            GPIO.output(pinGarageLight, GPIO.LOW)
            sleep(1)
            GPIO.output(pinSecurityLight, GPIO.LOW)
            CServo.set_servo(pinCServo, 1500)

def bedroomMotion(pinPIR_bedroom):
    if sm.securityStatus:
        if GPIO.input(pinPIR_bedroom):
            print('pinPIR_bedroom motion detected')
            GPIO.output(pinAlarm, GPIO.HIGH)
            GPIO.output(pinBedroomLight, GPIO.HIGH)
            sleep(5)
            CServo.set_servo(pinCServo, 2200)
            pubnub.publish().channel(SENSORS).message(MOTION_PIR1).async(my_publish_callback)
        else:
            print('pinPIR_bedroom motion detection ends')
            GPIO.output(pinAlarm, GPIO.LOW)
            GPIO.output(pinBedroomLight, GPIO.LOW)
            sleep(5)
            CServo.set_servo(pinCServo, 1500)
      
def sittingroomMotion(pinPIR_sittingroom):
    if sm.securityStatus:
        if GPIO.input(pinPIR_sittingroom):
            print('pinPIR_sittingroom motion detected')
            GPIO.output(pinAlarm, GPIO.HIGH)
            GPIO.output(pinSittingRoomLight, GPIO.HIGH)
            sleep(1)
            CServo.set_servo(pinCServo, 1200)
            pubnub.publish().channel(SENSORS).message(MOTION_PIR3).async(my_publish_callback)
        else:
            print('pinPIR_sittingroom motion detection ends')
            GPIO.output(pinAlarm, GPIO.LOW)
            GPIO.output(pinSittingRoomLight, GPIO.LOW)
            sleep(5)
            CServo.set_servo(pinCServo, 1500)
      
#sleep(2)
GPIO.add_event_detect(pinPIR_sittingroom, GPIO.BOTH, callback=sittingroomMotion)
GPIO.add_event_detect(pinPIR_bedroom, GPIO.BOTH, callback=bedroomMotion)
GPIO.add_event_detect(pinPIR_garage, GPIO.BOTH, callback=garageMotion)


#pubnub publish callback
def my_publish_callback(envelope, status):
    # Check whether request successfully completed or not
    if not status.is_error():
        pass 
        
    else:
        pass  # Handle message publish error. Check 'category' property to find out possible issue
        # because of which request did fail.
        # Request can be resent using: [status retry];


class MySubscribeCallback(SubscribeCallback):
    def status(self, pubnub, status):
        if status.operation == PNOperationType.PNSubscribeOperation \
                or status.operation == PNOperationType.PNUnsubscribeOperation:
            
            if status.category == PNStatusCategory.PNConnectedCategory:
                print('connected:::')
                p.ChangeDutyCycle(100)
                #GPIO.output(pinNetwork, GPIO.HIGH)
                pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
                pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)
                print(status.category)
                
            elif status.category == PNStatusCategory.PNReconnectedCategory:
                print('reconnected')
                
                p.ChangeDutyCycle(100)
                #GPIO.output(pinNetwork, GPIO.HIGH)
                pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
                pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)
                print(status.category)
                
            elif status.category == PNStatusCategory.PNDisconnectedCategory:
                print('not connected: disconected')
                sm.securityStatus = True
                p.ChangeDutyCycle(5)
                
                print(status.category)
                sleep(1)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNUnexpectedDisconnectCategory:
                print('not connected: unexpected disconnect')
                sm.securityStatus = True
                p.ChangeDutyCycle(5)
                #GPIO.output(pinNetwork, GPIO.LOW)
                print(status.category)
                sleep(1)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNAccessDeniedCategory:
                print('not connected: access denied')
                sm.securityStatus = True
                p.ChangeDutyCycle(5)
                #GPIO.output(pinNetwork, GPIO.LOW)
                print(status.category)
                
            elif status.category == PNStatusCategory.PNNetworkIssuesCategory:
                print('not connected: network issues')
                sm.securityStatus = True
                p.ChangeDutyCycle(5)
                #GPIO.output(pinNetwork, GPIO.LOW)
                print(status.category)
                sleep(1)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNTimeoutCategory:
                print('not connected: timeout')
                sm.securityStatus = True
                p.ChangeDutyCycle(5)
                #GPIO.output(pinNetwork, GPIO.LOW)
                print(status.category)
                sleep(1)
                pubnub.reconnect()    
            else:
                print('not connected: ')
                p.ChangeDutyCycle(5)
                print(status.category)
                sleep(1)
                pubnub.reconnect()
        elif status.operation == PNOperationType.PNSubscribeOperation:
            if status.is_error():
                print(status.operation)
                sleep(1)
                pubnub.reconnect()
            else:
                print(status.operation)
                # Heartbeat operation was successful
        else:
            #network_status()
            print(status.operation)
            # Encountered unknown status type
            
    def message(self, pubnub, message):

        payload = message.message
        print(message.channel)
        print (payload)

        if payload == securityLight_ON:
            GPIO.output(pinSecurityLight, GPIO.HIGH)
            pubnub.publish().channel(FEEDBACKS).message(securityLight_ON).async(my_publish_callback)
        
        elif payload == securityLight_OFF:
            GPIO.output(pinSecurityLight, GPIO.LOW)
            pubnub.publish().channel(FEEDBACKS).message(securityLight_OFF).async(my_publish_callback)
        
             
        elif payload == sittingRoomLight_ON:
            GPIO.output(pinSittingRoomLight, GPIO.HIGH)
            pubnub.publish().channel(FEEDBACKS).message(sittingRoomLight_ON).async(my_publish_callback)
        
        elif payload == sittingRoomLight_OFF:
            GPIO.output(pinSittingRoomLight, GPIO.LOW)
            pubnub.publish().channel(FEEDBACKS).message(sittingRoomLight_OFF).async(my_publish_callback)
        
        elif payload == bedroomLight_ON:
            GPIO.output(pinBedroomLight, GPIO.HIGH)
            pubnub.publish().channel(FEEDBACKS).message(bedroomLight_ON).async(my_publish_callback)
        
        elif payload == bedroomLight_OFF:
            GPIO.output(pinBedroomLight, GPIO.LOW)
            pubnub.publish().channel(FEEDBACKS).message(bedroomLight_ON).async(my_publish_callback)
        
            
        
        elif payload == camera_Right:
            #motionSensorData()getPosition(self)
            saveServoPosition.setPositionRight()
            
            CServo.set_servo(pinCServo, saveServoPosition.getPosition() )
            sleep(1)
            CServo.stop_servo(pinCServo)
            #pubnub.publish().channel(FEEDBACKS).message(camera_Right).async(my_publish_callback)
        
        elif payload == camera_Left:
            #motionSensorData()
            saveServoPosition.setPositionLeft()
            
            CServo.set_servo(pinCServo, saveServoPosition.getPosition() )
            sleep(1)
            CServo.stop_servo(pinCServo)
            #pubnub.publish().channel(FEEDBACKS).message(camera_Left).async(my_publish_callback)
        
        elif payload == garageDoor_OPEN:
            garageDoorServo.set_servo(pingarageDoorServo, 1600)
            sleep(1)
            garageDoorServo.stop_servo(pingarageDoorServo)
            #pubnub.publish().channel(FEEDBACKS).message(garageDoor_OPEN).async(my_publish_callback)
        
        elif payload == garageDoor_CLOSE:
            garageDoorServo.set_servo(pingarageDoorServo, 800)
            
            sleep(1)
            garageDoorServo.stop_servo(pingarageDoorServo)
            #pubnub.publish().channel(FEEDBACKS).message(garageDoor_CLOSE).async(my_publish_callback)
        
        elif payload == frontDoor_OPEN:
            fDoorServo.set_servo(pinFrontDoorServo, 2000)
            sleep(1)
            fDoorServo.stop_servo(pinFrontDoorServo)
            #pubnub.publish().channel(FEEDBACKS).message(frontDoor_OPEN).async(my_publish_callback)
        
        elif payload == frontDoor_CLOSE:
            fDoorServo.set_servo(pinFrontDoorServo, 800)
            sleep(1)
            fDoorServo.stop_servo(pinFrontDoorServo)
            #pubnub.publish().channel(FEEDBACKS).message(frontDoor_CLOSE).async(my_publish_callback)
        
        elif payload == changeMode_AUTO:
            sm.securityStatus = True
            sm.setSecurityStatus(sm.securityStatus)
            pubnub.publish().channel(FEEDBACKS).message(changeMode_AUTO).async(my_publish_callback)
        
        elif payload == changeMode_MANUAL:
            sm.securityStatus = False
            sm.setSecurityStatus(sm.securityStatus)
            pubnub.publish().channel(FEEDBACKS).message(changeMode_MANUAL).async(my_publish_callback)
        
        elif payload == sensors_Data:
            pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
            pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)
        
        elif message.channel == BEDROOM_LIGHT:
            b = int(payload)
            b = interp(b,[0,100],[0,100])
            bedroom.ChangeDutyCycle(b)
            
        elif message.channel == SITTINGROOM_LIGHT:
            s = int(payload)
            s = interp(s,[0,100],[0,100])
            sittingroom.ChangeDutyCycle(s)
            
        elif message.channel == SECURITY_LIGHT:
            se = int(payload)
            se = interp(se,[0,100],[0,100])
            securitylite.ChangeDutyCycle(se)
            
        elif message.channel == GARAGE_LIGHT:
            ga = int(payload)
            ga = interp(ga,[0,100],[0,100])
            garagelite.ChangeDutyCycle(ga)    
            
        elif message.channel == FAN:
            if payload == fan_ON:
                GPIO.output(pinRelay_fan, GPIO.LOW)
                pubnub.publish().channel(FEEDBACKS).message(fan_ON).async(my_publish_callback)
            elif payload == fan_OFF:
                GPIO.output(pinRelay_fan, GPIO.HIGH)
                pubnub.publish().channel(FEEDBACKS).message(fan_OFF).async(my_publish_callback)
            
                
            
    
    
    def presence(self, pubnub, presence):
        pass  # handle incoming presence data
    

pubnub.add_listener(MySubscribeCallback())
pubnub.subscribe().channels(MODE).execute()
pubnub.subscribe().channels(SENSORS).execute()
pubnub.subscribe().channels(SENSORS_TEMP).execute()
pubnub.subscribe().channels(SENSORS_HUMID).execute()
pubnub.subscribe().channels(FEEDBACKS).execute()
pubnub.subscribe().channels(CAMERA).execute()
pubnub.subscribe().channels(GARAGE_LIGHT).execute()
pubnub.subscribe().channels(DOORS).execute()
pubnub.subscribe().channels(SECURITY_LIGHT).execute()
pubnub.subscribe().channels(SITTINGROOM_LIGHT).execute()
pubnub.subscribe().channels(BEDROOM_LIGHT).execute() 
pubnub.subscribe().channels(PHOTO).execute()
pubnub.subscribe().channels(GARAGE_DOOR).execute()
pubnub.subscribe().channels(FRONT_DOOR).execute()
pubnub.subscribe().channels(FAN).execute()




class SecurityMode():
    securityStatus = False
    
    def setSecurityStatus(self, status):
        self.securityStatus = status
        
    def getSecurityStatus(self):
        return self.securityStatus
    
    
sm = SecurityMode()
print(sm.securityStatus)

class SaveServoPosition:
    defaultPosition = 1700
    
    def setPositionRight(self):
        if self.defaultPosition == 2300:
            self.defaultPosition = 2300
        else:
            self.defaultPosition += 100    
        
    def setPositionLeft(self):
        if self.defaultPosition == 600:
            self.defaultPosition = 600
        else:
            self.defaultPosition -= 100
        
    def getPosition(self):
        return self.defaultPosition
    
saveServoPosition =  SaveServoPosition()  

# function to read temperature and humidity
def read_temp_humidity():
    sleep(3)
    humidity, temperature = Adafruit_DHT.read_retry(temp_sensormodel, temp_sensorpin)
    print(humidity)
    print(temperature)
    return humid_value(humidity), temp_value(temperature) 



# get temp value
def temp_value(temp):
    temp = '{0:0.1f}'.format(float(temp))
    return temp

# get humidity value
def humid_value(humid):
    humid = '{0:0.1f}'.format(float(humid))
    return humid



def network_status():
    count = 0
    
    while count < 3:
        for dc in range(0, 101, 5):   # Increase duty cycle: 0~100
            p.ChangeDutyCycle(dc)     # Change duty cycle
            sleep(0.05)
        sleep(1)
        for dc in range(100, -1, -5): # Decrease duty cycle: 100~0
            p.ChangeDutyCycle(dc)
            sleep(0.05)
        sleep(1)
        count += 1



def continueScript():
    try:
        while True:
          sleep(0.3)    
        
    except KeyboardInterrupt:
        CServo.stop_servo(pinCServo)
        fDoorServo.stop_servo(pinFrontDoorServo)
        GPIO.cleanup()
        
        scriptProcess.terminate()
        sleep(1)
        sys.exit(1)
        

scriptProcess = multiprocessing.Process(target=continueScript, args=()).start()
sleep(1)

   
