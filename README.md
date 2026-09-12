<img width="1280" height="640" alt="git (1)" src="https://github.com/user-attachments/assets/8920b256-2ba8-4988-b824-5351134eb4bd" />



# Sooryakanthi


## Basic Details
### Team Name: inku minku


### Team Members
- Team Lead: Aalif Mohammad R S - college of engineering attingal
- Member 2: seethalekshmi G - College of engineering attingal


### Project Description
Indoor pan-tilt light pointer . Two hobby servos (yaw + pitch) and two LDRs with a vertical fin between them. An ESP32 DOIT DevKit V1 hunts the brightest spot in the room, holds lock, and lets a phone take over from a page hosted on the board.


### The Problem (that doesn't exist)
What if your room’s brightest spot was just slightly out of reach? This project solves the completely unnecessary crisis of manually pointing a light at the brightest part of the room

### The Solution (that nobody asked for)
Two LDRs act as the project’s tiny eyes, sensing which side is brighter. Two servos then swivel the light pointer left, right, up, or down until it finds the brightest spot. Once it locks on, you can either let the ESP32 keep hunting or take control from your phone through a web page hosted directly on the board. Basically: an unnecessarily intelligent light pointer that refuses to let the brightest spot escape.

## Technical Details
### Technologies/Components Used
For Software:
- Free rtos, platform io 
For Hardware:
- ESP32 
- servo motor
- LDR,3D printed enclousure
- Buck converter
- 12v adaptor
### Implementation
## Implementation

### Software

The ESP32 firmware was developed using **PlatformIO with the Arduino framework**. The two LDR sensors are connected to ADC1 pins and continuously measure the light intensity from the left and right sides. The difference between these readings is used to determine the direction in which the yaw servo should move.

Two servo motors control the **yaw and pitch** of the light pointer. In AUTO mode, the controller performs a search sequence to identify the brightest position and then enters LOCKED mode for fine tracking. A configurable deadzone prevents unnecessary servo movement when the LDR readings are already balanced.

The ESP32 also creates a Wi-Fi access point and hosts a control webpage. Communication between the webpage and firmware is handled using **WebSockets**, allowing real-time transmission of servo positions, LDR readings, and system status. Serial commands are also provided as a backup control interface.

### Installation

Clone the project repository and open it in VS Code with PlatformIO installed.


pio run
pio run -t upload


For the optional LittleFS webpage:

pio run -t uploadfs


### Run

1. Power the ESP32 and the separate servo supply.
2. Connect the phone to the **Sooryaganthi** Wi-Fi network.
3. Open the ESP32 control page at `192.168.4.1`.
4. Select **AUTO** to start light tracking.
5. Select **MANUAL** to control the yaw and pitch from the phone.
6. Use **Rescan** to force a new light-search sequence.
7. Cover both LDRs to demonstrate the **PARK** state.



### Project Documentation
## Project Summary

**Sooryakanthi** is an indoor pan-tilt light pointer built using an **ESP32 DOIT DevKit V1**, two hobby servo motors, and two LDR sensors. The system automatically searches for the brightest light source in a room by comparing the intensity detected by the left and right LDRs. A vertical fin between the sensors creates a differential light-sensing mechanism that helps determine the direction of the light source.

The system has three operating modes: **AUTO, MANUAL, and PARK**. In AUTO mode, the servos sweep the pan and tilt axes to locate the brightest position and then continuously fine-track it. In MANUAL mode, the user can control the yaw and pitch directly from a phone through a web interface hosted by the ESP32. PARK mode is activated when the environment becomes too dark, moving the pointer to a safe resting position until sufficient light is detected again.

The firmware uses **FreeRTOS** to separate the real-time control loop from Wi-Fi, HTTP, and WebSocket communication. This allows the light-tracking system to continue operating even if the phone connection becomes slow or disconnected. The project combines embedded systems, sensor-based control, servo positioning, Wi-Fi communication, and a simple web-based interface into one intentionally unnecessary but technically interesting system.


# Screenshots (Add at least 3)
![Screenshot1](Add screenshot 1 here with proper name)
*Add caption explaining what this shows*

![Screenshot2](Add screenshot 2 here with proper name)
*Add caption explaining what this shows*

![Screenshot3](Add screenshot 3 here with proper name)
*Add caption explaining what this shows*

# Diagrams
![Workflow](Add your workflow/architecture diagram here)
*Add caption explaining your workflow*

For Hardware:

# Schematic & Circuit
![Circuit](Add your circuit diagram here)
*Add caption explaining connections*

![Schematic](Add your schematic diagram here)
*Add caption explaining the schematic*

# Build Photos
![Components](Add photo of your components here)
*List out all components shown*

![Build](Add photos of build process here)
*Explain the build steps*

![Final](Add photo of final product here)
*Explain the final build*

### Project Demo
# Video
[Add your demo video link here]
*Explain what the video demonstrates*

# Additional Demos
[Add any extra demo materials/links]

## Team Contributions
- Aalif Mohammad R S: Hardware Assembly and design
- NAMEE: UI/UX

---
Made with ❤️ at TinkerHub Useless Projects 

![Static Badge](https://img.shields.io/badge/TinkerHub-24?color=%23000000&link=https%3A%2F%2Fwww.tinkerhub.org%2F)
![Static Badge](https://img.shields.io/badge/UselessProjects--26-26?link=https%3A%2F%2Ftinkerhub.org%2Fevents%2F1M8ORET9A1%2Fuseless-projects-3.0)



