import socket
import time
import select

# Configuration for both sending and receiving
sender_ip = "192.168.0.132"  # Replace with the sender's IP address
receiver_ip = "192.168.0.179"  # Replace with the receiver's IP address
send_port = 5002
receive_port = 5001

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a UDP socket for receiving data
receive_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the receiving socket to the sender's IP address and receive port
receive_sock.bind((sender_ip, receive_port))

# Set the receiving socket to non-blocking mode
receive_sock.setblocking(False)

position_values = [0, 50, 60]

while True:
    position_values[0] =  position_values[0] + 1
    # Get the current timestamp
    current_timestamp = time.perf_counter()

    # Prepare the message to send
    position_values_str = ','.join(map(str, position_values))
    msg = f"pos,{current_timestamp},{position_values_str}"  # Include "pos" and the timestamp at the beginning
    msg_bytes = msg.encode()
    
    # Send the message
    send_sock.sendto(msg_bytes, (receiver_ip, send_port))
    print(f"Sent message: {msg}")
    
     # This will read the buffer sequentialy
    """
    # Check for incoming data
    ready_to_read, _, _ = select.select([receive_sock], [], [], 0)
    
    if ready_to_read:
        data, addr = receive_sock.recvfrom(1024)  # Buffer size is 1024 bytes
        msg = data.decode()
        print(f"Received message: {msg} from {addr}")
    
    # Sleep for 1 millisecond
    time.sleep(1)
    """
    
    # This snippet will read the last value in the buffer
    # Check for incoming data
    latest_data = None
    while True:
        try:
            ready_to_read, _, _ = select.select([receive_sock], [], [], 0)  # Timeout of 0 seconds, non-blocking read
            if receive_sock in ready_to_read:
                latest_data, addr = receive_sock.recvfrom(1024)  # Buffer size is 1024 bytes
            else:
                break
        except Exception as e:
            print(f"Error: {e}")
            break

    if latest_data:
        msg = latest_data.decode()
        print(f"Received message: {msg} from {addr}")
    
    # Sleep for 2 seconds
    time.sleep(5)