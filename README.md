# NEO-PAC: A Pac-Man Solving Robot

NEO-PAC is a two part system that drives a physical robot through a real, built maze using the rules of Pac-Man. A PC acts as the "AI Brain": it watches the maze through a camera, detects walls, plans a safe path, and predicts ghost movement. The robot itself is a small differential drive car (ESP32 based) that receives movement commands over WiFi/UDP and executes them on the physical maze.

## How It Works

1. A camera looks down on the physical maze (built from foam/cardboard with grid markings).
2. The PC runs an OpenCV pipeline (assisted by the Claude Vision API) to detect walls and build a grid map of the maze.
3. The PC computes a route using A* pathfinding, with Theta* used to smooth the path into fewer, straighter segments, and a food routing algorithm to decide which food cells to visit and in what order.
4. The PC's Tkinter GUI shows the maze, the planned path, and lets the user start, stop, and switch modes.
5. The planned path is converted into a sequence of grid moves and sent to the robot over UDP.
6. The robot (ESP32 firmware in neopac_car.ino) executes the moves step by step, using PID assisted driving to keep the car tracking straight between grid cells, and reports acknowledgments and status back to the PC.

Ghost prediction was part of the original design but, along with YOLO based detection, live camera feed integration, and full PID tuning, was not completed. These remain open gaps shared across the team rather than the responsibility of any one member.

## System Architecture

PC side ("AI Brain")

* Wall detection: OpenCV pipeline combined with the Claude Vision API for maze/grid recognition.
* Path planning: A* search with Theta* path smoothing, plus a food routing algorithm to sequence food pickups.
* GUI: Tkinter based interface with a canvas for the maze, path overlays, and control buttons for mode switching.
* Glue code: builds the command/path messages and talks to the robot over the network.

Robot side (neopac_car.ino, ESP32)

* Locomotion Module: drives the L298N motor driver, with a lightweight PID assist that nudges left/right motor trim so the robot tracks straight while moving between grid cells.
* Command and Control Module: listens for commands over UDP, queues and executes them, and sends an acknowledgment (ACK) for every command plus status reports on request.
* Does not plan paths itself; it only executes the step by step moves it is given and reports back what it did.

The current firmware (v3.0) is a rebuild of an earlier ESP8266/HTTP prototype (v2). The board target moved from ESP8266 to ESP32, the transport moved from HTTP to UDP for lower latency, and ACK/status reporting, emergency stop, and speed/trim calibration were added on top of the original motor control logic.

## Communication Protocol

All messages are plain text, comma separated, and newline terminated. The PC sends commands to the robot's UDP port; the robot sends ACK/status replies back to the PC's UDP port.

| Message | Format | Purpose |
|---|---|---|
| Single command | CMD,<id>,<dir>[,<ms>] | Move one step in direction <dir> |
| Path (multi step) | PATH,<id>,<dir1>:<ms1>;<dir2>:<ms2>;... | Execute a full planned route |
| Speed | SPEED,<id>,<val 0 to 255> | Set base motor speed |
| Trim | TRIM,<id>,<left>,<right> | Calibrate left/right motor offset |
| Stop | STOP,<id> | Emergency stop, clears any running path |
| Status | STATUS,<id> | Request current path progress |

Direction codes: N (forward/north), D (backward/south), E (right/east), W (left/west), X (stop). The robot relative codes F, B, L, R are also accepted for manual testing.

Every command receives a reply of the form ACK,<id>,<result>. Status requests receive STAT,<id>,<status>,<step>,<total>,<remaining>.

## Hardware

* ESP32 development board
* L298N dual motor driver
* Two DC motors with wheels, plus battery pack
* Physical maze built from foam/cardboard with grid markings, designed to match the layout the PC's vision pipeline expects
* Overhead camera for the PC side vision pipeline

## Repository Contents

* neopac_car.ino: ESP32 firmware implementing the Locomotion and Command and Control modules described above.
* README.md: this file.
* (PC side Python code for vision, path planning, and the GUI lives alongside these files; see source files for details.)

## Team and Contributions

| Member | Roll No. | Contribution |
|---|---|---|
| Abdul Hanan | 24i 0087 | A* pathfinding, Theta* smoothing, and food routing algorithm (Python core logic) |
| Asjad Kamal | 24i 0046 | ESP8266 firmware: motor control, WiFi AP, HTTP routes, path execution (original v2 prototype) |
| Abdul Azeem | 24i 2013 | Tkinter GUI: canvas, overlays, buttons, mode switching |
| Abdul Rauf | 24i 0060 | Wall detection: OpenCV pipeline plus Claude Vision API integration; also built the physical maze (foam/cardboard structure, grid markings) |
| Fauzan Tahir | 24i 0042 | Glue code (car HTTP client, command generation), robot chassis assembly (wiring the L298N, motors, batteries), project proposal/report |

Known gaps: ghost prediction, YOLO based detection, full PID tuning, and live camera integration were not completed. These are shared gaps across the whole team rather than any single member's responsibility.

## Demo

Short clip of NEO-PAC running live: [Pacman_Live.mp4](https://github.com/AbdulAzeemHashmi/Pacman-RC-Car/blob/main/Pacman_Live.mp4)

## Project Proposal

Full project proposal document: [Project Proposal PDF](https://github.com/AbdulAzeemHashmi/Pacman-RC-Car/blob/main/Project-Proposal(AI)-24i0042%2C24i0046%2C24i0060%2C24i0087%2C24i2013.pdf)

## Status

This repository currently includes the rebuilt ESP32/UDP robot firmware (v3.0). The PC side AI Brain (vision, planning, and GUI) is developed separately and communicates with this firmware over the protocol described above.
