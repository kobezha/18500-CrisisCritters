# -*- coding: utf-8 -*-
import io
import time
import fcntl
import socket
import struct
from threading import Condition
import threading
from Led import *
from Servo import *
from Thread import *
from Buzzer import *
from Control import *
from ADC import *
from Ultrasonic import *
from Command import COMMAND as cmd

class Hexapod:
    def __init__(self):
        self.led=Led()
        self.adc=ADC()
        self.servo=Servo()
        self.buzzer=Buzzer()
        self.control=Control()
        self.sonic=Ultrasonic()
        self.control.Thread_conditiona.start()
  

    def process_instruction(self):
        while True:
            allData = input("Please enter a command: ")
            cmdArray=allData.split(' ')
            print(cmdArray)
            if cmdArray[-1] !="":
                cmdArray=cmdArray[:-1]
                
            for oneCmd in cmdArray:
                data=oneCmd.split("#")
                if data==None or data[0]=='':
                    continue
                elif cmd.CMD_BUZZER in data:
                    # > 0 is on
                    # 0 is off
                    self.buzzer.run(data[1])
                elif cmd.CMD_POWER in data:
                    try:
                        batteryVoltage=self.adc.batteryPower()
                        command=cmd.CMD_POWER+"#"+str(batteryVoltage[0])+"#"+str(batteryVoltage[1])+"\n"
                        print(command)
                        #self.send_data(self.connection1,command)
                        if batteryVoltage[0] < 5.5 or batteryVoltage[1]<6:
                         print("Battery Voltage is Low")
                         for i in range(3):
                            self.buzzer.run("1")
                            time.sleep(0.15)
                            self.buzzer.run("0")
                            time.sleep(0.1)
                    except:
                        pass
                elif cmd.CMD_LED in data:
                    try:
                        stop_thread(thread_led)
                    except:
                        pass
                    thread_led=threading.Thread(target=self.led.light,args=(data,))
                    thread_led.start()   
                elif cmd.CMD_LED_MOD in data:
                    try:
                        stop_thread(thread_led)
                        #print("stop,yes")
                    except:
                        #print("stop,no")
                        pass
                    thread_led=threading.Thread(target=self.led.light,args=(data,))
                    thread_led.start()
                elif cmd.CMD_SONIC in data:
                    command=cmd.CMD_SONIC+"#"+str(self.sonic.getDistance())+"\n"
                    self.send_data(self.connection1,command)
                elif cmd.CMD_HEAD in data:
                    if len(data)==3:
                        self.servo.setServoAngle(int(data[1]),int(data[2]))
                elif cmd.CMD_CAMERA in data:
                    if len(data)==3:
                        x=self.control.restriction(int(data[1]),50,180)
                        y=self.control.restriction(int(data[2]),0,180)
                        self.servo.setServoAngle(0,x)
                        self.servo.setServoAngle(1,y)
                elif cmd.CMD_RELAX in data:
                    #print(data)
                    if self.control.relax_flag==False:
                        self.control.relax(True)
                        self.control.relax_flag=True
                    else:
                        self.control.relax(False)
                        self.control.relax_flag=False
                elif cmd.CMD_SERVOPOWER in data:
                    if data[1]=="0":
                        GPIO.output(self.control.GPIO_4,True)
                    else:
                        GPIO.output(self.control.GPIO_4,False)
                elif cmd.CMD_STOPLED in data:
                    try:
                        stop_thread(thread_led)
                    except:
                        pass
                else:
                    self.control.order=data
                    self.control.timeout=time.time()

if __name__ == '__main__':
    hex = Hexapod()
    hex.process_instruction()
    
