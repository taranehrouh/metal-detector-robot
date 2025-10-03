# Metal-Detecting Robot (UBC ELEC 291)

A joystick-controlled **metal-detecting robot** with robust **radio communication** between a handheld controller and the robot. Two microcontrollers split the workload: the **controller** (master) handles UI, display, and audio feedback; the **robot** (slave) drives the wheels, performs metal sensing, and reports data back. All firmware is in **C**.

## Highlights
- **Two-MCU architecture:** PIC32MX130 (Controller) + STM32L051 (Robot)
- **Master–Slave radio protocol** using JDY-40 modules (UART)
- **Metal detection via Colpitts oscillator** (frequency shift thresholding)
- **Closed-loop drive control** (speed/direction from joystick)
- Extras: **Roomba Mode** (ultrasonic obstacle avoidance), **safety lights**, **speaker beeps** proportional to detection strength, optional **MP3 playback**

## System Overview
**Controller (Master)**
- Reads PS2-style joystick (2× analog axes @ 0–3.3 V)
- Sends speed/turn commands; requests metal-detector readings
- LCD shows detection strength; speaker beeps on detection

**Robot (Slave)**
- H-bridge motor drive (bidirectional control)
- Colpitts oscillator + coil → frequency measurement
- Ultrasonic distance sensing for Roomba Mode

**Comms (UART)**
- Typical setup: UART1 ≈ 9600 baud, UART2 ≈ 115200 baud  
- JDY-40 radio; fixed-length framing + simple filtering for noisy packets  
- Master polls; slave responds (single-value return) → low-lag control

## Hardware (Core)
| Subsystem            | Parts / Notes                                  |
|---------------------|--------------------------------------------------|
| Controller MCU       | PIC32MX130                                      |
| Robot MCU            | STM32L051                                       |
| Radio link           | JDY-40 (UART)                                   |
| Drive                | H-bridge (MOSFETs) + DC motors                  |
| Metal detector       | Coil + Colpitts oscillator → frequency reading  |
| UI / Feedback        | LCD (bar-style strength), speaker (beeps)       |
| Autonomy             | HC-SR04 ultrasonic sensor (Roomba Mode)         |
| Extras               | Safety lights (turn/brake), optional DFPlayer Mini MP3 |

## Repo Structure
