import socket

# Set the IP address and port of the CyBot server
HOST = "192.168.1.1"  # Replace with your CyBot's IP address
PORT = 288  # Replace with the appropriate port

# Create a TCP socket and connect to the CyBot server
cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cybot_socket.connect((HOST, PORT))

# Instructions for controlling the CyBot
print("Use 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right.")
print("Use 't' to toggle modes (Manual/Autonomous), 'h' to start scanning/moving in autonomous mode.")
print("Type 'quit' to exit.")

while True:
    command = input("Enter a command: ")  # Get movement command from the user

    # Send the command to the CyBot
    if command in ['w', 'a', 's', 'd', 't', 'h']:
        cybot_socket.sendall(command.encode())  # Send command to the CyBot

        # Receive and display the response from the CyBot
        data = cybot_socket.recv(1024).decode()
        if data:
            print(f"Received from CyBot: {data}")

    elif command == 'quit':
        cybot_socket.sendall(command.encode())  # Send quit command to CyBot
        break

    else:
        print("Invalid command. Use 'w', 'a', 's', 'd', 't', 'h', or 'quit'.")

# Close the socket connection
cybot_socket.close()
print("Connection closed.")




import socket
import numpy as np
import matplotlib.pyplot as plt

# Set the IP address and port of the CyBot server
HOST = "192.168.1.1"  # Replace with your CyBot's IP address
PORT = 288  # Replace with the appropriate port

# Create a TCP socket and connect to the CyBot server
cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cybot_socket.connect((HOST, PORT))

# Instructions for controlling the CyBot
print("Use 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right.")
print("Use 't' to toggle modes (Manual/Autonomous), 'h' to start scanning/moving in autonomous mode.")
print("Type 'm' to perform a 180-degree scan.")
print("Type 'quit' to exit.")

# Variables for storing scan data
angle_degrees = []
distance = []

def plot_polar(angle_degrees, distance):
    # Convert degrees to radians
    angle_radians = (np.pi / 180) * np.array(angle_degrees)

    # Create a polar plot
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.plot(angle_radians, distance, color='r', linewidth=2.0)

    ax.set_xlabel('Distance (cm)', fontsize=14.0)
    ax.set_ylabel('Angle (degrees)', fontsize=14.0)
    ax.xaxis.set_label_coords(0.5, 0.15)
    ax.tick_params(axis='both', which='major', labelsize=14)

    ax.set_rmax(max(distance) + 10)  # Set max radius slightly above max distance
    ax.set_rticks([10, 20, 30, 40, 50])  # Example tick marks, adjust as needed
    ax.set_rlabel_position(-22.5)
    ax.set_thetamax(180)
    ax.set_xticks(np.arange(0, np.pi + 0.1, np.pi / 4))
    ax.grid(True)

    ax.set_title("CyBot Sensor Scan from 0 to 180 Degrees", size=14, y=1.0, pad=-24)
    plt.show()

while True:
    command = input("Enter a command: ")  # Get movement command from the user

    # Send the command to the CyBot
    if command in ['w', 'a', 's', 'd', 't', 'h', 'm']:
        cybot_socket.sendall(command.encode())  # Send command to the CyBot

        if command == 'm':  # Start receiving scan data
            angle_degrees.clear()
            distance.clear()

            print("Receiving scan data from CyBot...")

            while True:
                data = cybot_socket.recv(1024).decode().strip()

                # Break loop if end of scan is detected (e.g., newline character)
                if not data or data == "\n":
                    break

                # Parse angle and distance data received from CyBot
                try:
                    angle, dist = map(int, data.split(','))
                    angle_degrees.append(angle)
                    distance.append(dist)
                    print(f"Angle: {angle}°, Distance: {dist} cm")  # Display received data
                except ValueError:
                    continue

            # Plot the collected data after receiving complete scan
            plot_polar(angle_degrees, distance)

    elif command == 'quit':
        cybot_socket.sendall(command.encode())  # Send quit command to CyBot
        break

    else:
        print("Invalid command. Use 'w', 'a', 's', 'd', 't', 'h', 'm', or 'quit'.")

# Close the socket connection
cybot_socket.close()
print("Connection closed.")