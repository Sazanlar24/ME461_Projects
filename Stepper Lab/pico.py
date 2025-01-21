import select
import sys
from time import sleep
from machine import Pin, ADC

class Stepper:
    def __init__(self, Pins):
        self.Pins = Pins
        self.poll_obj = select.poll()
        self.poll_obj.register(sys.stdin,1)
        self.state = None
        self.message = None
        self.order = 0
        self.ratio = 0
        
    # Serial communication via GUI
    def serialCom(self):
        if self.poll_obj.poll(0):
            self.message = sys.stdin.readline().strip() 
            if self.message.startswith('STATE'):
                self.state = self.message[6:]
    
    # One coil arrangment ignition. (i.e. '0 1 0 1')
    def igniteCoil(self, coil, delay=None):
        # Convert string into appropriate list of int. 
        coils = [int(coil[i]) for i in [0, 2, 4, 6]]
        
        # All pins are off. (i.e 0 0 0 0)
        for i in range(len(self.Pins)):
            self.Pins[i].value(0)
        
        # Ignite corresponsing coils. (i.e 0 1 0 1)
        for i in range(len(self.Pins)):
            self.Pins[i].value(coils[i])
        
        # If the the method is used in auto mode, wait for delay.
        if delay is not None:
            sleep(delay)
    
    def auto(self, seq, delay):
        self.order = 0
        self.ratio = 0
        
        # As soon as the state is auto or ratio, call auto method.
        while self.state in ['AUTO', 'RATIO']:
            # Check if there is any change in state or message. 
            self.serialCom()
            # Iterate over each coil arrangment in sequence list in an infinite manner.
            # Using the particular coil arrangment, ignite them one by one with delay. 
            self.order %= len(seq)
            coil = seq[self.order]
            self.igniteCoil(coil, delay)
            
            self.order += 1
            self.ratio += 1
        
        # Print the gearbox ratio by stopping the stepper after one full revolution.
        print(f"Ratio: {self.ratio/64}")
    
    # All pins are off in case of stop signal.
    def stop(self):
        for pin in self.Pins:
            pin.off()

# Assign pins and store them in a list
ANeg = Pin(14, Pin.OUT)
APos = Pin(15, Pin.OUT)
BNeg = Pin(16, Pin.OUT)
BPos = Pin(17, Pin.OUT)
pins= [ANeg, APos, BNeg, BPos]

# Create stepper object
myStepper = Stepper(pins)

while True:
    # Half-stepping sequence: 1 0 0 0, 1 0 0 1, 0 0 0 1, 0 1 0 1, 0 1 0 0, 0 1 1 0, 0 0 1 0, 1 0 1 0      
    # Iterate the loop by checking any message from GUI
    myStepper.serialCom()
    
    # Depending on the state of the stepper, call corresponding method of the stepper
    if myStepper.state == 'MANUAL':
        # Extract coil arrangement and ignite as it is. (i.e. '0 1 0 1')
        if myStepper.message[:4] == 'COIL' :
            coil = myStepper.message[5:len(myStepper.message)]
            myStepper.igniteCoil(coil)
            
    if myStepper.state == 'AUTO':
        # Extract delay (sec) between consecutive steps in auto mode.
        if myStepper.message[:5] == 'DELAY' :
            delay = float(myStepper.message[6:len(myStepper.message)])/1000
        
        # Extract coil sequence for auto mode. (seq coming from GUI is like ''1 0 0 0-1 0 0 1 -0 0 0 1 -0 1 0 1-0 1 0 0-0 1 1 0-0 0 1 0-1 0 1 0-')
        elif myStepper.message[:3] == 'SEQ' :
            seq = myStepper.message[4:len(myStepper.message)-1]
            # Convert a list of coil arrangments by '-'. Then delete any whitespaces at the end
            # and elminate the last element which is '-'. Finally, call corresponding method.
            seq = seq.split('-')
            seq = [x.rstrip() for x in seq]
            seq = seq[:len(seq)-1]
            myStepper.auto(seq, delay)
    
    if myStepper.state == 'RATIO':
        # Any delay and valid coil sequence for turning the stepper.
        delay = 2/1000
        seq = '1 0 0 0-1 0 0 1 -0 0 0 1 -0 1 0 1-0 1 0 0-0 1 1 0-0 0 1 0-1 0 1 0-'
        seq = seq.split('-') 
        seq = seq[:len(seq)-1]
        myStepper.auto(seq, delay)
        
    if myStepper.state == 'STOP':
        myStepper.stop()
    
    sleep(0.1)   