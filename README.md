Design a simple PID to control a robot head
===========================================

# Prerequisites
You are supposed to know something on what a [PID controller](https://en.wikipedia.org/wiki/PID_controller) can do for you and why it is so versatile and effective :wink:

If that's not the case yet, here's below an _eclectic incomplete not-at-all curated list_ of quick or semi-quick refreshers on the argument:
- [PID for dummies](https://www.csimn.com/CSI_pages/PIDforDummies.html)
- [The basics of tuning PID loops](https://innovativecontrols.com/blog/basics-tuning-pid-loops)
- [How should I tune my copter?](https://github.com/betaflight/betaflight/wiki/PID-Tuning-Guide)
- [The good SE resource](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)
- [PID controller tuning in Simulink](https://it.mathworks.com/help/slcontrol/gs/automated-tuning-of-simulink-pid-controller-block.html)
- [Did you already take on these awesome control challenges?](https://janismac.github.io/ControlChallenges)

Some academic resources (take them more seriously :smirk:):
- [PID control system analysis, design, and technology](https://doi.org/10.1109/TCST.2005.847331)
- [The Bible of the PID](https://aiecp.files.wordpress.com/2012/07/1-0-1-k-j-astrom-pid-controllers-theory-design-and-tuning-2ed.pdf)

But stop reading and loafing around now :hammer: and heat up the skills that made you a great PID human-tuner :muscle:

# Assignment
There is a robot standing in a world far away, whose body is fully anchored to the ground. Fortunately, the robot can still gaze around. There is also a magic red ball sticking out from the beautiful landscape. The ball floats insanely in front of the robot, moving along unpredictable trajectories and even teleporting from one place to another.

Given that the robot observes the world through its **320x240** couple of cameras, your task is "simply" :laughing: to **let the robot look constantly at the floating ball**, as fast as possible, controlling both the eyes and the neck.

Thereby, you have to **tune a bunch of PID velocity controllers** that will drive the joint motors and in turn we will assign you points incrementally, as you meet the two sets of requirements below.

#### R1. Requirements to satisfy when the ball jumps between locations

1. The _x pixel coordinate_ of the ball in the _left camera image_ shall approach the center: **ul ≈ 160**.
1. The _y pixel coordinate_ of the ball in the _left camera image_ shall approach the center: **vl ≈ 120**.
1. The _x pixel coordinate_ of the ball in the _right camera image_ shall approach the center: **ur ≈ 160**.
1. The _y pixel coordinate_ of the ball in the _right camera image_ shall approach the center: **vr ≈ 120**.
1. The robot shall keep the _eyes tilt_ close to 0 degrees: **eyes-tilt ≈ 0 [deg]**.
1. The robot shall keep the _eyes pan_ close to 0 degrees: **eyes-pan ≈ 0 [deg]**.

#### R2. Requirements to satisfy when the ball moves at constant speed

1. The _x pixel coordinate_ of the ball in the _left camera image_ shall approach the center: **ul ≈ 160**.
1. The _y pixel coordinate_ of the ball in the _left camera image_ shall approach the center: **vl ≈ 120**.

#### Score map

| Requirements | Points |
|:---:|:---:|
| R1.1 | 4 |
| R1.2 | 1 |
| R1.3 | 4 |
| R1.4 | 1 |
| R1.5 | 2 |
| R1.6 | 2 |
| R2.1 | 8 |
| R2.2 | 4 |

The maximum score you can achieve is therefore **26** :trophy:

## Expected output

If you'll do your job correctly, the outcome should look like the animation below :sunglasses:

![output](/misc/output.gif)

## How to proceed

We provide you with a starter code that contains missing gaps you have to fill in. Don't panic, most of the software to detect the ball and deal with the module infrastructure is already done. **Just focus on the control part**, possibly refining and extending what you'll find therein.

Once done, you can test your code in two ways:

1. **Manually**: running the _yarpmanager scripts_ provided from within [**app/scripts**](./app/scripts). This will help you tune the PID gains smoothly.
1. **Automatically**: [running the script **test.sh**](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-run-smoke-tests.md) in the **smoke-test** directory. This will give you an idea of how many points you might score.

To better inspect the quality of your control, you can plot the position of the ball in the image planes along with the angles of the eyes joints as they evolve over time.

We can achieve that using [Octave](https://www.gnu.org/software/octave/). Just launch:

```sh
$ ./plot.sh
```
You will obtain temporal diagrams like this one :sparkles:

![profiles](/misc/profiles.png)

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
