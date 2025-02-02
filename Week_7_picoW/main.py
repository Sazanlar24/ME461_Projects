from machine import Pin, ADC
import utime

# Setup LED for blink and heartbeat
led = Pin('LED', Pin.OUT)

# Setup temperature sensor
sensor_temp = ADC(4)
conversion_factor = 3.3 / (65535)

def led_blink():
    count = 0
    number = input("Total number of blinks: ")
    duration = round(float(input('Duration of one blink in ms: ')))/1000
    ratio = 0.5
    
    if number == 'inf':
        numberLimit = 1
    else: numberLimit = int(number)
    
    while count < numberLimit:        
        led.on() 
        utime.sleep(ratio*duration)  
        led.off()
        utime.sleep((1-ratio)*duration)
        
        if number != 'inf':
            count += 1

def heartbeat():
    count = 0
    number = input("Total number of blinks: ")
    duration = round(float(input('Duration of one blink in ms: ')))/1000
    freq = 100
    
    if number == 'inf':
        numberLimit = 1
    else: numberLimit = int(number)
    
    while count < numberLimit:  
        ratio = 0.9
        cycle_time = duration/freq
        light_up = ratio*cycle_time
        light_down = (1-ratio)*cycle_time
        
        for i in range(freq):
            led.on()
            on_time = light_up*(i/freq)
            utime.sleep(on_time)
            led.off()
            utime.sleep(light_up-on_time)
        
        for i in range(freq):
            led.off()
            off_time = light_down*(i/freq)
            utime.sleep(off_time)
            led.on()
            utime.sleep(light_down-off_time)
    
        if number != 'inf':
            count += 1
        
        
def calculator():
    while True: 
        text = input("Enter calculation text: ")

        try:
            result = eval(text)
            print("Result:", result)
            continue
        except:
            print('The evaluation could not be operated! Try it again!')
            continue
        

def display_temperature():
    while True:
        reading = sensor_temp.read_u16() * conversion_factor
        temperature = 27 - (reading - 0.706) / 0.001721
        print("Temperature: {:.2f}°C".format(temperature))
        utime.sleep(1/3)

def reverse_text():
    while True:
        text = input("Enter text to reverse: ")
        reversed_text = ''
        for i in range(len(text)):
            reversed_text += text[-(i+1)] 
        print("Reversed Text:", reversed_text)

def main():
    while True:
        print("Select an option:")
        print("1. LED Blink")
        print("2. Heart Beat")
        print("3. Calculator")
        print("4. Display Onboard Temperature Reading")
        print("5. Reverse the Given Text")
        selection = input("Enter your selection: ")

        try: 
            if selection == '1':
                led_blink()
            elif selection == '2':
                heartbeat()
            elif selection == '3':
                calculator()
            elif selection == '4':
                display_temperature()
            elif selection == '5':
                reverse_text()
            else:
                print("Invalid selection, please try again.")
        
        except KeyboardInterrupt:
            continue
        
if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt:
        main()
