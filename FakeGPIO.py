import threading
import time
import socket
import json
from collections import deque

# I will simulate 


class stepper:
    def __init__(self, enabled, direction, steps):        
        self.enabled = enabled
        self.direction = direction
        self.steps = steps

    def set_direction(self, direction):
        if direction == 'GPIO.HIGH':
            self.direction = 1
        else:
            self.direction = -1

    def step_pulse(self, pulse):
        # If the port is known and value is HIGH, consider it as 1 step
        if self.enabled and pulse == 'GPIO.HIGH':
                self.steps += self.direction

    def set_enabled(self, enabled):
        self.enabled = enabled
            
    
class FakeGPIO:

    # Constants
    OUT = 'GPIO.OUT'
    HIGH = 'GPIO.HIGH'
    LOW = 'GPIO.LOW'
    BCM = 'GPIO.BCM'
    IN = 'GPIO.IN'
    PUD_UP = 'GPIO.PUD_UP'

    # Known stepper and switch constants
    known_ports = [
        26, 19, 13, 6, 5, 21, 20, 16, 12, 1, 17, 4, 3, 2
    ]

    # Stepper and switch constants
    stepper_bottom_enable = 26
    stepper_bottom_step = 19
    stepper_bottom_dir = 13
    stepper_bottom_ms1 = 6
    stepper_bottom_ms2 = 5

    stepper_top_enable = 21
    stepper_top_step = 20
    stepper_top_dir = 16
    stepper_top_ms1 = 12
    stepper_top_ms2 = 1

    limit_switch_top = 17
    limit_switch_bottom = 4
    limit_switch_left = 3
    limit_switch_right = 2
    
    def __init__(self):        
        self.step_queue = deque()
        self.queue_consumer_thread = threading.Thread(target=self.consume_queue)
        self.queue_consumer_thread.start()
        self.stepper_bottom = stepper(False, 1, 0)
        self.stepper_top = stepper(False, 1, 0)
        self.limit_switch_bottom_state = False
        self.limit_switch_top_state = False
        self.limit_switch_left_state = False
        self.limit_switch_right_state = False

    previous_Xsteps = -1000
    previous_Ysteps = -1000

    def getYsteps(self):
        return (self.stepper_bottom.steps + self.stepper_top.steps) 
    
    def getXsteps(self):
        return (self.stepper_top.steps - self.stepper_bottom.steps) 

    def update_limit_switchs(self):
        # Update limit switch states
        self.limit_switch_bottom_state = self.getYsteps() > 21250
        self.limit_switch_top_state = self.getYsteps() < -21250
        self.limit_switch_left_state = self.getXsteps() < -28750 
        self.limit_switch_right_state = self.getXsteps() > 28750

    def setwarnings(self, flag):
        # Mimic the behavior or log
        print(f"Set warnings called with flag = {flag}")

    def setmode(self, mode):
        # Mimic the behavior or log
        print(f"Set mode called with mode = {mode}")

    def setup(self, port, direction, pull_up_down=None):
        pass  # Normally, this is where you'd configure GPIO pins. Empty for this example.

    def output(self, port, value):        
        if port == self.stepper_bottom_step:
            self.stepper_bottom.step_pulse(value)
            self.update_limit_switchs()
        elif port == self.stepper_top_step:
            self.stepper_top.step_pulse(value)
            self.update_limit_switchs()
        elif port == self.stepper_bottom_dir:
            self.stepper_bottom.set_direction(value)
        elif port == self.stepper_top_dir:
            self.stepper_top.set_direction(value)
        elif port == self.stepper_bottom_enable:
            self.stepper_bottom.set_enabled(value==0)
        elif port == self.stepper_top_enable:
            self.stepper_top.set_enabled(value==0)
        else:
            print(f"Output called with port = {port} and value = {value}")  
        
    def input(self, port):
        # Mimic the behavior or log
        if port == self.limit_switch_bottom:
            return 0 if self.limit_switch_bottom_state else 1
        elif port == self.limit_switch_top:
            return 0 if self.limit_switch_top_state else 1
        elif port == self.limit_switch_left:
            return 0 if self.limit_switch_left_state else 1
        elif port == self.limit_switch_right:
            return 0 if self.limit_switch_right_state else 1
        else:
            print(f"Input called with port = {port}")
        return 0

    def consume_queue(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8765))
        try:

            while True:
                message = ""
                if self.previous_Xsteps != self.getXsteps() or self.previous_Ysteps != self.getYsteps():
                    message = f"{self.getXsteps()},{self.getYsteps()}"
                    self.previous_Xsteps = self.getXsteps()
                    self.previous_Ysteps = self.getYsteps()
                if message != "":
                    client_socket.sendall(message.encode())                

                time.sleep(0.2)  # wait for 0.5 seconds

        except KeyboardInterrupt:
            print("Stopping the queue consumer thread")
            client_socket.close()
            return
        
        finally:
            client_socket.close()
        
            

# Let's assume this function is imported from another module
def send_data_to_server(data):
    print(f"Sending data to server: {data}")

