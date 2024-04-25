#!/usr/bin/python3

import sys
import math
import threading
import signal
import socket
import struct
import rclpy
import time
import json
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension

class Msg_to_send:
    def __init__(self, grid, num_rows, num_cols, row_stride, col_stride):
        self.grid = grid
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.row_stride = row_stride
        self.col_stride = col_stride


verbose = True

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('hexapod_control_node')
        self.message_pub = self.create_publisher(Int32MultiArray, "/message_received", 10)

        self.keep_running = True

        # Set up signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'message_send',
            self.listener_callback,
            10)  

        self.server_thread = threading.Thread(target=self.server)
        self.server_thread.start()

        self.client_thread = threading.Thread(target=self.client)
        self.client_thread.start()

        self.msg_to_send = None
        self.finished_sending = True
        self.send_msg = False


    def listener_callback(self, msg):
        if verbose: self.get_logger().info(f'Entered Message Send Listener Callback')
        
        self.msg_to_send = Msg_to_send([], 0, 0, 0, 0)
        self.msg_to_send.grid = msg.data
        for dim in msg.layout.dim:
            if dim.label == "rows":
                self.msg_to_send.num_rows = dim.size
                self.msg_to_send.row_stride = dim.stride
            if dim.label == "cols":
                self.msg_to_send.num_cols = dim.size
                self.msg_to_send.col_stride = dim.stride

        self.finished_sending = False
        self.send_msg = True

        while not self.finished_sending and self.keep_running:
            self.msg_to_send.grid = msg.data
            for dim in msg.layout.dim:
                if dim.label == "rows":
                    self.msg_to_send.num_rows = dim.size
                    self.msg_to_send.row_stride = dim.stride
                if dim.label == "cols":
                    self.msg_to_send.num_cols = dim.size
                    self.msg_to_send.col_stride = dim.stride

        
        self.send_msg = False
        self.msg_to_send = None


    def server(self):
        listensocket = socket.socket() # Creates an instance of socket
        Port = 8000 # Port to host server on
        maxConnections = 999
        IP = socket.gethostname() # IP address of local machine
        
        listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listensocket.bind(('',Port))

        # Starts server
        listensocket.listen(maxConnections)
        listensocket.settimeout(1)  # Set a timeout of 1 second

        if verbose: self.get_logger().info(f"Server started at {IP}")
        
        # Accepts the incoming connection
        accepted = False
    
        while self.keep_running and not accepted:
            try:
                clientsocket, clientAddress = listensocket.accept()
                clientsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                if verbose: self.get_logger().info(f"Accepted connection from: {clientAddress}")

                clientsocket.settimeout(5) # IMP: Set a timeout of 1 second
                accepted = True
    
                while self.keep_running and accepted:
                    try:
                        data = clientsocket.recv(5000)  # Gets the incoming messagea
               
                        decoded_data = data.decode()
                        
                        if not decoded_data:
                            if verbose: self.get_logger().info("Client Timed Out")
                            accepted = False
                            clientsocket.close()
                        else:
                            data = json.loads(decoded_data)
                            is_heartbeat = data.get("is_heartbeat")
                            
                            if not is_heartbeat:
                                grid = data.get("grid")
                                num_rows = data.get("num_rows")
                                row_stride = data.get("row_stride")
                                num_cols = data.get("num_cols")
                                col_stride = data.get("col_stride")
        
                                msg = Int32MultiArray()
                                msg.data = grid
                                
                                # Create layout for a 2D array
                                layout = MultiArrayLayout()
                                layout.dim.append(MultiArrayDimension(label="rows", size=num_rows, stride=row_stride))
                                layout.dim.append(MultiArrayDimension(label="cols", size=num_cols, stride=col_stride))
                                msg.layout = layout
                                
                                if verbose: self.get_logger().info(f"message is {msg.data}")
                                self.message_pub.publish(msg)
                               
                    except Exception as e:
                        if verbose: self.get_logger().info(f"Client Timed Out\t Error is {e}")
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
                # Creates instance of 'Socket'
                s = socket.socket()

                # IMP: Hostname should be IP address of other hexapod || Change appropriately
                hostname = '172.26.17.218' # Server Hostname
                port = 8000 # Server Port
    
    
                try:
                    s.connect((hostname,port)) # Connects to server
                    connected = True
                    if verbose: self.get_logger().info("Client connected")

                except:
                    if not self.keep_running:
                        if verbose: self.get_logger().info("Client shutdown")
                        s.close()
                        return
            
            # IMP: Can be changed but sleeps for 1 second between consecutive messages
            time.sleep(1)

            msg  = json.dumps({"is_heartbeat": True, "grid": [], "num_rows": 0, "num_cols": 0, "row_stride": 0, "col_stride": 0}) 

            if self.send_msg:
                msg  = json.dumps({"is_heartbeat": False, "grid": list(self.msg_to_send.grid), "num_rows": self.msg_to_send.num_rows, "num_cols": self.msg_to_send.num_cols, "row_stride": self.msg_to_send.row_stride, "col_stride": self.msg_to_send.col_stride}) 

            
            try:
                s.send(msg.encode()) # Encodes and sends message
            except:
                connected = False

            if self.send_msg:
                self.finished_sending = True
        
        s.close()
        if verbose: self.get_logger().info("Client shutdown")


    def signal_handler(self, sig, frame):
        if verbose: self.get_logger().info("Ctrl+C detected. Shutting down...")
        self.keep_running = False
        self.server_thread.join()
        self.client_thread.join()
        sys.exit(0)

if __name__ == '__main__':
    rclpy.init(args=None)
    communication_node = CommunicationNode()
    rclpy.spin(communication_node)
    communication_node.destroy_node()
    rclpy.shutdown()

