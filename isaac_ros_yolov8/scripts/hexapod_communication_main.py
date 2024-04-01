#!/usr/bin/python3

import sys
import math
import threading
import time
from Client import *
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ControlMsgSubscriber(Node):
    def __init__(self):
        super().__init__('hexapod_control_node')
        self.subscription = self.create_subscription(
            String,
            'hexapod_commands',
            self.listener_callback,
            10)  
        self.hexapod = HexapodNode()
        self.hexapod.connect()
        signal.signal(signal.SIGINT, self.hexapod.signal_handler)

    def listener_callback(self, msg):
        self.get_logger().info(f'Entered ListenerCallback')
        if msg.data == "turn_right":
            self.hexapod.turn_right()
        elif msg.data == "turn_right_90":
            self.hexapod.turn_right_90()
        elif msg.data == "turn_left":
            self.hexapod.turn_left()
        elif msg.data == "move_forward":
            self.hexapod.move_forward()
        elif msg.data == "move_backward":
            self.hexapod.move_backward()
        elif msg.data == "move_left":
            self.hexapod.move_left()
        elif msg.data == "move_right":
            self.hexapod.move_right()
        elif msg.data == "stop_moving":
            self.hexapod.stop_moving()
        elif msg.data == "start_buzzing":
            self.hexapod.start_buzzing()
        elif msg.data == "stop_buzzing":
            self.hexapod.stop_buzzing() 
        elif msg.data == "get_ready":
            self.hexapod.get_ready()
        elif msg.data == "relax":
            self.hexapod.relax()      

        

class HexapodNode(Node):
    def __init__(self):
        super().__init__('hexapod_node')
        self.client= Client()
        # file = open('IP.txt', 'r')
        # self.IP = str(file.readline())
        # file.close()
        self.IP = "172.26.178.42"
        self.cmd = "stop_moving"
        self.shutdown = threading.Event()
        

    def turn_left(self):
        command = "CMD_MOVE#1#-33#0#9#-10\n"
        self.client.send_data(command) 
    
    def turn_right(self):
        command = "CMD_MOVE#1#33#0#9#10\n"
        self.client.send_data(command)   
    
    def turn_right_90(self):
        command = "CMD_MOVE#1#33#0#9#11\n"
        self.client.send_data(command)
        time.sleep(3)
        command = "CMD_MOVE#1#0#0#9#0\n"
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
            
            self.get_logger().info("Connection Successful!") 

        except Exception as e:
            print ("Connect to server Faild!: Server IP is right? Server is opend?")
            self.client.tcp_flag=False
        while not self.shutdown.is_set():
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
        
    #CONNECT
    def connect(self):
        self.client.turn_on_client(self.IP)
        self.instruction_thread=threading.Thread(target=self.receive_instruction,args=(self.IP,))
        self.instruction_thread.daemon = True
        self.instruction_thread.start()  

    #BUZZER
    def start_buzzing(self):
        command=cmd.CMD_BUZZER+'#1'+'\n'
        self.client.send_data(command) 
        
    
    def stop_buzzing(self):
        command=cmd.CMD_BUZZER+'#0'+'\n'
        self.client.send_data(command) 
        
        
    # Handler for Ctrl+C
    def signal_handler(self, sig, frame):
        print("Ctrl+C pressed. Exiting...")
        self.stop_moving()
        self.stop_buzzing() 
        self.client.turn_off_client()
        self.shutdown.set()
        self.instruction_thread.join()
        sys.exit(0)
        

if __name__ == '__main__':
    rclpy.init(args=None)
    control_msg_subscriber = ControlMsgSubscriber()
    rclpy.spin(control_msg_subscriber)
    control_msg_subscriber.destroy_node()
    rclpy.shutdown()
    

        
