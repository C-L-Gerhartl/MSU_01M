# MSU_01M

Arduino Sketch to control NUC start/shutdown behavior.

This repository contains an Arduino sketch to control the start and shutdown behavior of a NUC-PC running a Linux-based operating system.  
Additionally, you need to install the Python script `vehicle_shutdown.py` on the NUC.
Logic is shown in StateMaschine.xlsx... 

---

## 🚀 Getting Started

### Arduino Setup

1. Clone this repository.
2. Download and install the [Arduino IDE](https://www.arduino.cc/en/softwareduino IDE.
4. Connect your Arduino and flash the sketch.

### Linux Mint Setup (NUC)

5. Copy the Python script `vehicle_shutdown.py` to your home directory:
   ```bash
6. Make sure Python 3 is installed: python3 --version
7. Make the script executable: chmod +x ~/vehicle_shutdown.py
8. Copy the systemd service file to the correct location: sudo cp vehicle_shutdown.service /etc/systemd/system/
9. Edit the service file to match your username and script path: sudo nano /etc/systemd/system/vehicle_shutdown.service
10. Enable and start the service: sudo systemctl daemon-reexec
                                  sudo systemctl daemon-reload
                                  sudo systemctl enable vehicle-shutdown.service
                                  sudo systemctl start vehicle-shutdown.service
11. Check the service status: sudo systemctl status vehicle-shutdown.service

## 📬 Contact
Developed by Christian Stein
Feel free to reach out for collaborations, feedback, or suggestions!
