# Spectral-BLDC

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)  ![Issues](https://img.shields.io/github/issues/PCrnjak/Spectral-BLDC-Python) ![release](https://img.shields.io/github/v/release/PCrnjak/Spectral-BLDC-Python)

Python lib for controlling [spectral BLDC](https://github.com/PCrnjak/Spectral-Micro-BLDC-controller/blob/main/README.md) controllers and [SSG-48 gripper](https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper) over CAN bus. 
For more info about this API and all available commands check [DOCS](https://source-robotics.github.io/Spectral-BLDC-docs/apage7_can/)

# How to install

  pip install Spectral-BLDC

# Basic example


``` py 
import Spectral_BLDC as Spectral
import time


Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)

while True:

    Motor1.Send_Respond_Encoder_data()

    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=0.2)

    if message is not None:

        Motor1.UnpackData(message,UnpackedMessageID)
        print(f"Motor position is: {Motor1.position}")
        print(f"Motor speed is: {Motor1.speed}")

    else:
        print("No message after timeout period!")
    print("")
    time.sleep(1 )
```


# More examples

Check out the Examples folder for more examples!



This project is entirely open-source and free for all to use. Any support, whether through donations or advice, is greatly appreciated. Thank you!

 [![General badge](https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white)](https://paypal.me/PCrnjak?locale.x=en_US)
[![General badge](https://img.shields.io/badge/Patreon-F96854?style=for-the-badge&logo=patreon&logoColor=white)](https://www.patreon.com/PCrnjak)
