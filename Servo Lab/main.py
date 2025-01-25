import uselect
import sys
from machine import ADC, Pin, PWM
from time import sleep

spoll = uselect.poll()
spoll.register(sys.stdin, uselect.POLLIN)

# POT connected to GPIO27
pot_pin = ADC(Pin(27)) 

class Servo:
    def __init__(self, pin_number):
        self.servo = PWM(Pin(pin_number))
        self.servo.freq(50)
        
        self.pwm_0 = 1350
        self.pwm_180 = 7900
        
        self.angle = 0
        self.prev_angle = 0
        self.buffer = ""
        
        self.released = False
        
        # Moving average filter variables
        self.window_size = 5
        self.readings = []
        
    def evaluate_pot(self):
        # Read ADC value
        pot_value = pot_pin.read_u16()
        
        # Update readings list for moving average
        self.readings.append(pot_value)
        if len(self.readings) > self.window_size:
            self.readings.pop(0)
        
        # Apply moving average
        smoothed_value = sum(self.readings) / len(self.readings)
        
        # Map smoothed value to 0-180 degrees
        mapped = 180 * smoothed_value / 65535
        return mapped
    
    def set_angle(self, angle):
        # Set the servo angle.
        #:param angle: Angle in degrees (0 to 180)
        if (0 <= angle <= 180):
            duty = int(self.pwm_0 + angle * (self.pwm_180 - self.pwm_0) / 180)
            self.servo.duty_u16(duty)
        else:
            return

    def read_from_usb(self):
        while True:
            if spoll.poll(0):  # Check if there's input
                c = sys.stdin.read(1)  # Read one character
                if c == "\n":  # Check for end-of-line (Enter key)
                    self.buffer = self.buffer.strip()
                    if self.buffer.upper() == "RELEASE":
                        self.released = True
                    else:
                        self.released = False
                        try:
                            self.angle = int(self.buffer)
                        except:
                            pass
                        self.buffer = ""  # Clear buffer for the next input
                else:
                    self.buffer += c  # Add character to buffer
            
            try:
                if self.released:
                    pot_angle = self.evaluate_pot()
                else:
                    pot_angle = int(self.angle)
            
            except:
                pass
            
            if self.prev_angle != pot_angle:
                self.set_angle(pot_angle)
            
            self.prev_angle = pot_angle
            
my_servo = Servo(28)
my_servo.read_from_usb()


