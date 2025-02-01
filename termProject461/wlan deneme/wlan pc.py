import socket

# Pico W IP address and port
HOST = '192.168.4.1'
PORT = 8080

try:
    # Connect to the Pico W server
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HOST, PORT))
    print(f"Connected to {HOST}:{PORT}")
except socket.error as e:
    print(f"Failed to connect to {HOST}:{PORT}. Error: {e}")
    exit(1)  # Exit the program if the connection fails

try:
    while True:
        try:
            # Get user input
            #Example usage: PIX:1,2,green /// 1:First half of neopixel, 2:seconds (0 for infinity), green:color
            #Example usage: MOTOR:1,8000,True /// 1:Motor1, 8000:Duty cycle (max 65025), True:Direction
            #Example usage: TURN:LEFT,65025, 0.45 /// LEFT:Turn Right, 65025:Duty cycle, 0.45: turn for 0.45 secs (for 65025 duty, left->0.45sec, right->0.38sec 90 degrees)
            message = input("Enter command (e.g., PIX:1,2,green): ")
            print(f'Message sent: {message}')
            
            # Validate the message before sending (optional, implement as needed)
            if not message.strip():
                print("Empty message. Please enter a valid command.")
                continue

            # Send data to the server
            client.sendall(message.encode())
            print("Message sent.")

            '''# Receive and process the response
            try:
                response = client.recv(1024)
                if not response:
                    print("No response received. Server might have disconnected.")
                    #break
                print("Received from Pico:", response.decode())
            except :
                print(f"Error receiving data!")'''
        
        except KeyboardInterrupt:
            print("\nTerminating connection.")
            break
        except Exception as e:
            print(f"An error occurred: {e}")
except:None
'''finally:
    # Close the connection safely
    try:
        client.close()
        print("Connection closed.")
    except socket.error as e:
        print(f"Error closing connection: {e}")
'''