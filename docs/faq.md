# FAQ

Questions people often ask about this project, with short answers.

## General

**What does the robot do?**
It finds blue and red cubes, drives to them, picks them up with its arm and drops them in a basket on its back, then looks for the next one.

**What hardware is it?**
A Hiwonder ArmPi Pro: a 6-servo arm on a mecanum-wheel base, run by a Raspberry Pi 4. It has two cameras: one on the arm and an ACEBOTT K210 AI camera at the front.

**What software does it use?**
Python, ROS Melodic on Ubuntu 18.04, OpenCV for image processing, and YOLO-World on a laptop for AI vision.

**Is it fully autonomous?**
Yes, once started it searches, grabs and repeats on its own until no cube is left.

## Vision

**How does it recognise the cube?**
By colour: it keeps only blue or red pixels, then checks the patch is shaped like a cube (solid, square-ish, big enough).

**What if something else is blue?**
The shape check filters out most things. YOLO-World was added because colour alone was fooled by dark-blue clothing.

**Why two cameras?**
The front camera finds the cube from a distance while driving. The arm camera looks straight down for exact lining up.

**What is YOLO-World and why use it?**
An AI model that finds objects from a text description (like "blue square") without training on that object. It is not fooled by colour alone, and the target can be changed by changing the words.

**Why does the AI run on the laptop?**
The Raspberry Pi is too slow for it; the laptop is about 10 times faster. The laptop only sends the position; the robot still moves the arm itself.

**What happens if the laptop disconnects?**
The arm holds still and the robot gives up after a while instead of moving blindly.

## Arm and movement

**How does the arm know where to go?**
Inverse kinematics: given a target position (reach and height), it calculates each joint's angle.

**How does it line up exactly over the cube?**
A feedback loop: look, move a little toward the centre, look again, until the cube stays centred for 10 pictures.

**Why does the arm shake before grabbing?**
The camera moves with the arm, so each correction changes the picture and can overshoot; the detected position also jitters slightly. See [troubleshooting](troubleshooting.md).

**What if the cube is rotated?**
The wrist turns to match the cube's angle before grabbing.

**What if it misses?**
It tries again (up to 8 attempts per cube; with the grip check turned on, up to 3 grabs on the spot).

## Problems and limits

**What was the hardest part?**
Changing light breaking colour detection, lining the gripper up precisely, and the battery: when it is low, the arm sags and grabs land in the wrong place.

**Why does the battery matter so much?**
Low voltage makes the servos weak and caused the Pi to restart during sudden wheel starts. The code refuses to drive below 7.5 V.

**What doesn't work yet?**
The robot side of the AI mode and the self-learning grab point have not been tested on the robot yet. The grab doesn't succeed every time.

**Can it avoid obstacles?**
No, it has no distance sensor. Driving modes should run in open floor space.

## Future

**Could it pick up other objects?**
With YOLO-World, in principle yes, by changing the text prompt. The gripper size is the limit.

**What would you improve next?**
Test AI mode on the robot, drive using AI vision, and use a bigger model that understands instructions like "pick up the thing next to the cup".

**Where could this be used?**
Warehouse sorting, tidying robots, recycling sorting, picking parts in a factory.
