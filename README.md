# Autonomous Maze-Navigating Robot

## Overview
A robot built for Science Olympiad that navigates mazes using onboard sensors. 

## Team members
William Shih (mentor) 
Jeremiah Laing (team lead & CAD lead) 
CJ Frausto (software lead) 
Kodee Garcia (wiring lead) 
Joshua Situ (software) 
Justin Thai (mechanical) 

## My role
- Led a 5-person team, including delegating tasks, scheduling meetings, and competition execution
- Designed the CAD model in OnShape
- Soldered and designed the wiring
- Component integration and purchasing
- Testing, troubleshooting, iterating
- Collaborated on software development 

## Demo
Coming soon

## Mechanical Design
- Chassis designed in OnShape
- Two 12V brushed DC motors (126 RPM with 64-quadrature encoder)
- Two 6cm diameter wheels
- One 360-degree turning wheel (from model airplane) 
- 8 rechargeable 1.2V NiMH AA batteries
- Balanced between efficient shape for maze interaction (water bottle handling) and having a compact footprint

## Electrical System
- 34 solid core wire connections soldered to protoboard shield
- Arduino Mega
- TB6612FNG Motor Driver
- 12V-5V voltage regulator
- Solved loose connections by transitioning to soldering solid-core wires

## How It Works
- Compass reads heading
- Robot decides direction based on compass and time data
- User inputs high-level movement commands 
- Motors execute movement
- U-shaped body supports turning while holding a water bottle

## Competition Constraints
- 30x30 cm footprint
- 4 water bottles
- 5 target zones
- Target time ranges between 55 and 70 seconds

## Iterations
Version 0: 
- 2025-2026 robot
- Designed for previous rules
- Acted as the foundation for our design

Version 1 - 12/24/2025 
- First U-Shape model
- Frame was too thin and buckled under weight of the components
- Forgot to CAD holes for the wheels, motors, and component mounts (batteries had to be taped) 

Version 2 - 1/1/2026
- Fixed the buckling issue with crossbeam and increased thickness
- Added support for screw holes on both sides (motors sit directly beneath batteries)
- Added an inward-sloping opening to "catch" the water bottles

Version 3 - 1/23/2026
- Significantly redesigned frame for practicality, robustness, and aesthetics
- Added tabs at the back for motor wire management
- Added holes for on-button and power switch mounting (previously, they sat on the breadboard)
- Facilitated transition to solid-core wires by resizing the raised platform 

Version 4 - 4/3/2026
- Opted for notch inside the frame, instead of tabs on outside, for superior wire management
- Included dedicated section for dowel to be superglued on the inside (determines start and end positions according to the rules) 

## Future Improvements
- Fix the compass
- Improve time-based decision-making
- Replace the wheels and motors 
