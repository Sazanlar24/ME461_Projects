from machine import Pin
import time

# Configure GPIO pin as input with an internal pull-up resistor
button_R = Pin(14, Pin.IN, Pin.PULL_UP)
button_L = Pin(13, Pin.IN, Pin.PULL_UP)

last_press_time_left = 0
last_press_time_right = 0

encoder_mode = "X1"
encoder_count = 0

outer_sensor = Pin(16, Pin.IN)  # GP16 as input
inner_sensor = Pin(15, Pin.IN)  # GP15 as input

# Callback function
def sensor_inner_callback(pin):
    global encoder_count
    if pin.value() == 0:
        print("White region detected by inner sensor (rising edge)")
        if (encoder_mode == "X1" or encoder_mode == "X2"):
            encoder_count += 1
        elif (encoder_mode == "X4"):
            outer = outer_sensor.value()
            if outer==0:
                print("CW rotation")
                encoder_count += 1
            else:
                print("CCW rotation")
                encoder_count += 1
    else:
        print("Black region detected by inner sensor (falling edge)")
        if (encoder_mode == "X2"):
            encoder_count += 1
        elif (encoder_mode == "X4"):
            outer = outer_sensor.value()
            if outer==0:
                print("CCW rotation")
                encoder_count += 1
            else:
                print("CW rotation")
                encoder_count += 1
    print(encoder_count)            
        
def buttonR_pressed_callback(pin):
    global last_press_time_right, encoder_count
    current_time = time.ticks_ms()  # Get current time in milliseconds
    if time.ticks_diff(current_time, last_press_time_right) > 300:  # 200 ms debounce
        print("Encoder count is reseted.")
        encoder_count = 0
        last_press_time_right = current_time

# Callback function with debouncing for right
def buttonL_pressed_callback(pin):
    global last_press_time_left, encoder_mode
    current_time = time.ticks_ms()  # Get current time in milliseconds
    if time.ticks_diff(current_time, last_press_time_left) > 300:  # 300 ms debounce
        if (encoder_mode == "X1"):
            encoder_mode = "X2"
        elif (encoder_mode == "X2"):
            encoder_mode = "X4"
        elif (encoder_mode == "X4"):
            encoder_mode = "X1"
        print(f"Encoder mode is changed. New mode is: {encoder_mode}")
        last_press_time_left = current_time


# Configure the GPIO pin to trigger an interrupt on both rising and falling edges
inner_sensor.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=sensor_inner_callback)

# Callback function with debouncing for buttons
button_R.irq(trigger=Pin.IRQ_FALLING, handler=buttonR_pressed_callback)
button_L.irq(trigger=Pin.IRQ_FALLING, handler=buttonL_pressed_callback)

# Main loop
try:
    while True:
        time.sleep(1)  # Idle loop
except KeyboardInterrupt:
    print("Program stopped.")
