#!/usr/bin/python3

import sys
import math
import threading
import signal
import socket
import struct
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
#from isaac_ros_yolov8.msg import CustomGrid


verbose = True

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('hexapod_control_node')
        self.message_pub = self.create_publisher(String, "/message_recieved", 10)

        self.keep_running = True

        # Set up signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        self.subscription = self.create_subscription(
            String,
            'message_send',
            self.listener_callback,
            10)  

        #self.server_thread = threading.Thread(target=self.server)
       #self.server_thread.start()

        self.client_thread = threading.Thread(target=self.client)
        self.client_thread.start()

        self.msg_to_send = ""
        self.finished_sending = True
        self.send_msg = False


    def listener_callback(self, msg):
        if verbose: self.get_logger().info(f'Entered ListenerCallback')
        
        self.msg_to_send = msg.data
        self.finished_sending = False
        self.send_msg = True

        while not self.finished_sending and self.keep_running:
            self.msg_to_send = msg.data
        
        self.send_msg = False


    def server(self):
        listensocket = socket.socket() #Creates an instance of socket
        Port = 8000 #Port to host server on
        maxConnections = 999
        IP = socket.gethostname() #IP address of local machine
        
        listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listensocket.bind(('',Port))

        #Starts server
        listensocket.listen(maxConnections)
        listensocket.settimeout(1)  # Set a timeout of 1 second

        if verbose: self.get_logger().info(f"Server started at {IP}")
        #Accepts the incoming connection
        accepted = False
    
        while self.keep_running and not accepted:
            try:
                clientsocket, clientAddress = listensocket.accept()
                clientsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                if verbose: self.get_logger().info(f"Accepted connection from: {clientAddress}")

                clientsocket.settimeout(10) # Set a timeout of 1 second
                accepted = True
    
                while self.keep_running and accepted:
                    try:
                        data = clientsocket.recv(5000)  # Gets the incoming message

                        msg = String()
                        msg.data = data.decode()
    
                        if len(msg.data) > 0:
                            if verbose: self.get_logger().info(f"message is {msg.data}")
                            self.message_pub.publish(msg)
                        else:
                            if verbose: self.get_logger().info("Client Timed Out")
                            accepted= False
                            clientsocket.close()

                    except Exception as e:
                        if verbose: self.get_logger().info("Client Timed Out")
                        accepted = False
                        clientsocket.close()

                continue 
    
            except socket.timeout as e:
                if not self.keep_running:
                    # If keep_running is False, it means we're shutting down, so ignore the exception
                    listensocket.close()  # Close the server socket

                    if verbose: self.get_logger().info("Server Shutdown")

                    return
                continue

        clientsocket.close()  # Close the client socket
        listensocket.close()  # Close the server socket
        if verbose: self.get_logger().info("Server Shutdown")


    def client(self):
        connected = False
    
        while self.keep_running:
    
            while not connected:
                #Creates instance of 'Socket'
                s = socket.socket()
                #hostname = '172.26.165.27' #Server IP/Hostname
                #hostname = '192.168.8.30' #Server IP/Hostname

                #for the communication target
                hostname = '172.26.17.218'
                #hostname = '96.236.202.36'
                port = 8000 #Server Port
    
    
                try:
                    s.connect((hostname,port)) #Connects to server
                    connected = True
                    if verbose: self.get_logger().info("Client connected")

                except:
                    if not self.keep_running:
                        if verbose: self.get_logger().info("Client shutdown")
                        s.close()
                        return
            
            time.sleep(1)

            msg = "heartbeat"
            if self.send_msg:
                msg = self.msg_to_send
            
            try:
                s.send(msg.encode()) #Encodes and sends message (x)
            except:
                connected = False

            if self.send_msg:
                self.finished_sending = True
        
        s.close()
        if verbose: self.get_logger().info("Client shutdown")


    def signal_handler(self, sig, frame):
        if verbose: self.get_logger().info("Ctrl+C detected. Shutting down...")
        self.keep_running = False
        #self.server_thread.join()
        self.client_thread.join()
        sys.exit(0)

if __name__ == '__main__':
    rclpy.init(args=None)
    communication_node = CommunicationNode()
    rclpy.spin(communication_node)
    communication_node.destroy_node()
    rclpy.shutdown()



#
#keep_running = True
#
#def server():
#    global keep_running
#    listensocket = socket.socket() #Creates an instance of socket
#    Port = 8000 #Port to host server on
#    maxConnections = 999
#    IP = socket.gethostname() #IP address of local machine
#    
#    listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#    listensocket.bind(('',Port))
#
#    #Starts server
#    listensocket.listen(maxConnections)
#    listensocket.settimeout(1)  # Set a timeout of 1 second
#    print("Server started at " + IP + " on port " + str(Port))
#
#    #Accepts the incoming connection
#    accepted = False
#
#    while keep_running and not accepted:
#        try:
#            clientsocket, clientAddress = listensocket.accept()
#            clientsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#            print("Accepted connection from: ", clientAddress)
#            clientsocket.settimeout(10) # Set a timeout of 1 second
#            accepted = True
#
#            while keep_running and accepted:
#                try:
#                    data = clientsocket.recv(1024)  # Gets the incoming message
#                    message = data.decode()
#
#                    if message:
#                        print(message)
#
#
#                except Exception as e:
#                    print(e)
#                    accepted = False
#            continue 
#
#        except socket.timeout as e:
#            if not keep_running:
#                # If keep_running is False, it means we're shutting down, so ignore the exception
#                listensocket.close()  # Close the server socket
#                print("Server shutdown")
#                return
#            continue
#
#
#    clientsocket.close()  # Close the client socket
#    listensocket.close()  # Close the server socket
#    print("Server shutdown")
#
#def client():
#    global keep_running
#    #Creates instance of 'Socket'
#    s = socket.socket()
#
#    hostname = '172.26.31.240' #Server IP/Hostname
#
#    port = 8000 #Server Port
#
#    connected = False
#
#
#    s.connect((hostname,port)) #Connects to server
#
#
#    while not connected:
#        try:
#            s.connect((hostname,port)) #Connects to server
#            connected = True
#        #except ConnectionRefusedError as e:
#        except:
#            if not keep_running:
#                print("Client shutdown")
#                s.close()
#                return
#
#    while keep_running:
#        x = input("Enter Command:") #Gets the message to be sent
#        s.send(x.encode()) #Encodes and sends message (x)
#
#    s.close()
#
#    print("Client shutdown")
#
#def signal_handler(sig, frame):
#    global keep_running
#    print('Ctrl+C detected. Shutting down...')
#    keep_running = False
#
#if __name__ == '__main__':
##   client_thread= threading.Thread(target=client)
##   client_thread.start()
#   server_thread= threading.Thread(target=server)
#   server_thread.start()
#
#    # Set up signal handler for Ctrl+C
#   signal.signal(signal.SIGINT, signal_handler)
#
##   client_thread.join()
#   server_thread.join()
#   sys.exit(0)
