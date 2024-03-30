import Spectral_BLDC as Spectral
import time

# Init comms channel 
Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
# Init motor object
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)

while True:

    # Send this msg to the motor controller, it will respond with position and speed data
    Motor1.Send_Respond_Encoder_data()

    # Get CAN msg, timeout tells us to continue if there is no msg after in this case 0.2 s
    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=0.2) 

    # If there was a msg unpack it
    if message is not None:
        print(f"Message is: {message}")
        print(f"Node ID is : {UnpackedMessageID.node_id}")
        print(f"Message ID is: {UnpackedMessageID.command_id}")
        print(f"Error bit is: {UnpackedMessageID.error_bit}")
        print(f"Message length is: {message.dlc}")
        print(f"Is is remote frame: {message.is_remote_frame}")
        print(f"Timestamp is: {message.timestamp}")
        
        # Here we unpack the data from the packet
        Motor1.UnpackData(message,UnpackedMessageID)
        # We are interested in position and speed data that was requested by calling Send_Respond_Encoder_data
        print(f"Motor position is: {Motor1.position}")
        print(f"Motor speed is: {Motor1.speed}")


    else:
        print("No message after timeout period!")
    print("")
    time.sleep(1 )
