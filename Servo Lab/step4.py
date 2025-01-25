# python script to apply REPL input to servo
import sys
from machine import ADC, Pin, PWM
from time import sleep

# Connected to GPIO28
servo = PWM(Pin(28))
servo.freq(50)  # 50 Hz frequency

pwm_0 = 1350
pwm_180 = 7900

def set_angle(angle):
    # Set the servo angle.
    if 0 <= angle <= 180:
        duty = int(pwm_0 + angle * (pwm_180 - pwm_0) / 180)
        servo.duty_u16(duty)

while True:
    message = sys.stdin.readline().strip()  # Read a line of input from REPL
    try:
        if message != "":
            pot_angle = int(message)
            set_angle(pot_angle)  # Only call if conversion succeeds
    except ValueError:
        print("Invalid input, try again.")
    sleep(0.01)

