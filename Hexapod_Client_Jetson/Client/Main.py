# -*- coding: utf-8 -*-
import sys
import math
import threading
from Client import *
class HexapodNode():
    def __init__(self):
        self.client= Client()
        file = open('IP.txt', 'r')
        self.IP = str(file.readline())
        file.close()
        self.cmd = "stop_moving"

    def turn_right(self):
        command = "CMD_MOVE#1#28#0#9#10\n"
        self.client.send_data(command)        
    
    def turn_left(self):
        command = "CMD_MOVE#1#-33#0#9#10\n"
        self.client.send_data(command)     
    
    def stop_moving(self):
        command = "CMD_MOVE#1#0#0#9#0\n"
        self.client.send_data(command)  
        
    def move_forward(self):
        command = "CMD_MOVE#1#0#29#9#0\n"
        self.client.send_data(command)  
        
    def move_backward(self):
        command = "CMD_MOVE#1#0#-33#9#0\n"
        self.client.send_data(command) 
        
    def move_left(self):
        command = "CMD_MOVE#1#-33#0#9#0\n"
        self.client.send_data(command)  
        
    def move_right(self):
        command = "CMD_MOVE#1#32#0#9#0\n"
        self.client.send_data(command)  

    def relax(self):
        command = cmd.CMD_SERVOPOWER + "#" + "0" + '\n'
        self.client.send_data(command)
            
    def get_ready(self):
        command = cmd.CMD_SERVOPOWER + "#" + "1" + '\n'
        self.client.send_data(command)
            
    def close_client(self):
        stop_thread(self.instruction_thread)
        stop_thread(self.get_command_thread)
        self.client.turn_off_client()
        sys.exit(0)

    def power(self):
        try:
            command = cmd.CMD_POWER + '\n'
            self.client.send_data(command)
            self.progress_Power1.setFormat(str(self.power_value[0])+"V")
            self.progress_Power2.setFormat(str(self.power_value[1]) + "V")
            self.progress_Power1.setValue(self.restriction(round((float(self.power_value[0]) - 5.00) / 3.40 * 100), 0, 100))
            self.progress_Power2.setValue(self.restriction(round((float(self.power_value[1]) - 7.00) / 1.40 * 100), 0, 100))
            #print (command)
        except Exception as e:
            print(e)

    def receive_instruction(self,ip):
        try:
            self.client.client_socket1.connect((ip,5002))
            self.client.tcp_flag=True
            print ("Connecttion Successful !")
        except Exception as e:
            print ("Connect to server Faild!: Server IP is right? Server is opend?")
            self.client.tcp_flag=False
        while True:
            try:
                alldata=self.client.receive_data()
            except:
                self.client.tcp_flag=False
                break
            #print(alldata)
            if alldata=='':
                break
            else:
                cmdArray=alldata.split('\n')
                #print(cmdArray)
                if cmdArray[-1] !="":
                    cmdArray==cmdArray[:-1]
            for oneCmd in cmdArray:
                data=oneCmd.split("#")
                print(data)
                if data=="":
                    self.client.tcp_flag=False
                    break
                elif data[0]==cmd.CMD_SONIC:
                    self.label_sonic.setText('Obstacle:'+data[1]+'cm')
                    print('Obstacle:',data[1])
                elif data[0]==cmd.CMD_POWER:
                    try:
                        if len(data)==3:
                            self.power_value[0] = data[1]
                            self.power_value[1] = data[2]
                            self.power_value[0] = self.restriction(round((float(data[1]) - 5.00) / 3.40 * 100),0,100)
                            self.power_value[1] = self.restriction(round((float(data[2]) - 7.00) / 1.40 * 100),0,100)
                            print('Power：',power_value1,power_value2)
                    except Exception as e:
                        print(e)
                        
    def get_commands(self):
        while True:
            print("Commands are:\nturn_right\nturn_left\nmove_forward\nmove_backward\nmove_right\nmove_left\nstop_moving")
            user_input = input("Enter Command:")
            if user_input == "turn_right":
                self.cmd = "turn_right"
            elif user_input == "turn_left":
                self.cmd = "turn_left"
            elif user_input == "move_forward":
                self.cmd = "move_forward"
            elif user_input == "move_backward":
                self.cmd = "move_backward"
            elif user_input == "move_left":
                self.cmd = "move_left"
            elif user_input == "move_right":
                self.cmd = "move_right"
            elif user_input == "stop_moving":
                self.cmd = "stop_moving"
            elif user_input == "relax":
                self.cmd = "stop_moving"
                self.relax()
            elif user_input == "get_ready":
                self.get_ready()
            elif user_input == "start_buzzing":
                self.start_buzzing()
            elif user_input == "stop_buzzing":
                self.stop_buzzing()
            elif user_input == "close_client":
                self.close_client()
            else:
                self.cmd = "stop_moving"
        
    #CONNECT
    def connect(self):
        self.client.turn_on_client(self.IP)
        self.instruction_thread=threading.Thread(target=self.receive_instruction,args=(self.IP,))
        self.instruction_thread.start()  
        self.get_command_thread=threading.Thread(target=self.get_commands) 
        self.get_command_thread.start()

    #BUZZER
    def start_buzzing(self):
        command=cmd.CMD_BUZZER+'#1'+'\n'
        self.client.send_data(command) 
    
    def stop_buzzing(self):
        command=cmd.CMD_BUZZER+'#0'+'\n'
        self.client.send_data(command) 

if __name__ == '__main__':
    hexapod = HexapodNode()
    hexapod.connect()
    
    while True:
        if hexapod.cmd == "turn_right":
            hexapod.turn_right()
        elif hexapod.cmd == "turn_left":
            hexapod.turn_left()
        elif hexapod.cmd == "move_forward":
            hexapod.move_forward()
        elif hexapod.cmd == "move_backward":
            hexapod.move_forward()
        elif hexapod.cmd == "move_left":
            hexapod.move_left()
        elif hexapod.cmd == "move_right":
            hexapod.move_right()
        elif hexapod.cmd == "stop_moving":
            hexapod.stop_moving()
        else:
            hexapod.stop_moving()
        
        