# Team 9520 Autonomy Simulator — User Guide for Students

Welcome! This guide explains, step by step and in plain language, how to use the Team 9520 autonomy simulator. You will learn how to draw a path for the robot, run it, understand what went right or wrong, practice with realistic problems (like a slippery floor or a bumped robot), and finally put your work on the real robot.

You do not need to know the robot code to use this guide. Every special word is explained the first time it appears, and all of them are collected in [Words you will see](#2-words-you-will-see). If you get stuck, look at [Troubleshooting](#12-troubleshooting-and-common-questions).

> **Tip:** Keep this guide open in one window and the simulator in another, and try each thing as you read about it.

---

## Contents

1. [What the simulator is and why it helps](#1-what-the-simulator-is-and-why-it-helps)
2. [Words you will see](#2-words-you-will-see)
3. [Before you start (one-time setup)](#3-before-you-start-one-time-setup)
4. [Starting and stopping the simulator](#4-starting-and-stopping-the-simulator)
5. [A tour of the screen](#5-a-tour-of-the-screen)
6. [The Plan tab: drawing paths and building autos](#6-the-plan-tab-drawing-paths-and-building-autos)
7. [The Run tab: running and understanding what happened](#7-the-run-tab-running-and-understanding-what-happened)
8. [Practice scenarios](#8-practice-scenarios)
9. [Watching in AdvantageScope and Elastic](#9-watching-in-advantagescope-and-elastic)
10. [Switching between the simulator and the real robot](#10-switching-between-the-simulator-and-the-real-robot)
11. [Putting your paths and autos on the real robot](#11-putting-your-paths-and-autos-on-the-real-robot)
12. [Troubleshooting and common questions](#12-troubleshooting-and-common-questions)
13. [Where to learn more](#13-where-to-learn-more)

---

## 1. What the simulator is and why it helps

In a FIRST Robotics Competition match, the first 20 seconds are the **autonomous period**: the robot drives and scores entirely on its own, with no driver touching the controls. The instructions it follows during those 20 seconds are called an **auto** (short for "autonomous routine"). Writing a good auto is hard, because you usually only get a few minutes with the real robot and a real field to test it.

The simulator solves that problem. It runs **the real robot program** — the exact same Java code that goes on the robot — on your laptop instead of on the robot. The laptop pretends to be the robot's motors, sensors and cameras, and a web page shows you a top-down picture of the 2026 field with the robot driving on it.

The most important idea in the simulator is the difference between two robots that you will see on the screen at the same time:

- **The truth** (a solid robot with a green outline) is where the robot *really* is on the field.
- **The estimate** (a dashed blue outline) is where the robot's program *thinks* it is.

On a real robot you can only ever see the truth with your eyes, and the program only ever knows its estimate. The two slowly drift apart because wheels slip and sensors are never perfect. The simulator shows you both at once, so you can see exactly how far off the robot's belief is, why, and how the cameras help pull it back. You can also see the **plan** (the path you drew) and the **setpoint** (where the robot is supposed to be right now), so you can tell whether a problem comes from the path itself, from the way the robot follows it, or from the robot not knowing where it is.

---

## 2. Words you will see

These are listed in alphabetical order. Come back here whenever you meet a word you are not sure about.

| Word | What it means |
|---|---|
| **Alliance** | One of the two teams of three robots in a match, either **blue** or **red**. The field is mirrored, so a red-alliance auto is the blue one turned around. |
| **AprilTag** | A square black-and-white printed marker, like a simple barcode, fixed at known places on the field. A camera that sees one can work out where the robot is. [WPILib: AprilTag introduction](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html) |
| **Auto** | Short for "autonomous routine": the list of steps the robot does by itself in the autonomous period. In this project, an auto is a file ending in `.auto`. |
| **Auto Chooser** | A drop-down menu on the driver's dashboard where you pick which auto the robot will run in the next match. |
| **Autonomous period** | The first 20 seconds of a 2026 match, when robots act on their own. |
| **AdvantageScope** | A free program for watching robot data live or from saved logs, with graphs and a field view. [AdvantageScope documentation](https://docs.advantagescope.org/overview/live-sources/) |
| **Bézier handle** | The small dots on lines sticking out of a waypoint. Dragging them bends the curve of the path, the same way handles work in drawing programs. |
| **BUMP** | A low ramp on the 2026 field (about 6.5 inches tall) that robots drive over. There are two next to each HUB. |
| **CTRE (Cross The Road Electronics)** | The company that makes our motor controllers, the Pigeon gyro and the Tuner X setup program. |
| **CAN (Controller Area Network)** | The wiring network inside the robot that connects the main computer to the motor controllers and sensors. Every device has a number called its **CAN ID**. |
| **Command / named command** | A command is one action the robot can do (for example "spin up the shooter"). A **named command** is a command that has been given a name in the robot code so you can use it in your autos. |
| **Constraints** | Speed limits for a path: the fastest the robot may drive, accelerate and turn. |
| **DEPOT** | A floor area along each alliance wall where FUEL is stored. |
| **Deploy** | Copying the robot program from your laptop onto the robot's main computer. |
| **Drift** | The slow, growing difference between where the robot really is and where it thinks it is, caused by wheel slip and imperfect sensors. |
| **Driver Station** | The program on the drivers' laptop (Windows only) that connects to the robot, enables and disables it, and chooses teleoperated or autonomous mode. [WPILib: Driver Station](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html) |
| **Elastic** | The dashboard program our drivers use to see robot information and to pick the auto. [Elastic documentation](https://frc-elastic.gitbook.io/docs) |
| **Enable / disable** | Turning the robot's motors on or off from the Driver Station. A disabled robot cannot move. |
| **Estimate** | Where the robot's program *believes* the robot is. Shown as a dashed blue outline. |
| **Event marker** | A point along a path where the robot starts a named command *while it keeps driving* (for example, "start the intake here"). |
| **Field of view** | How wide a camera can see, measured as an angle. A camera with an 80-degree field of view sees a wedge 80 degrees wide. |
| **Flywheel** | The heavy spinning wheel in the shooter that throws the FUEL. It has to reach full speed before a ball is fed in. |
| **FIRST (For Inspiration and Recognition of Science and Technology)** | The organization that runs the FIRST Robotics Competition. |
| **FRC (FIRST Robotics Competition)** | The high-school robotics competition this robot is built for. |
| **FUEL** | The 2026 game piece: a foam ball, about 6 inches across. A robot may start a match holding up to 8. |
| **Gradle** | The tool that builds (compiles) the robot program and copies it to the robot. You run it with the `./gradlew` command. |
| **Gyro (Pigeon 2)** | A sensor inside the robot that measures which way the robot is facing. Ours is a Pigeon 2 made by the company CTRE. |
| **Ethernet** | A wired network cable, like the one you plug into a home internet router. |
| **Heading** | The direction the robot is facing, measured in degrees. On this field, 0 degrees faces the red alliance wall and 90 degrees faces the left side wall (as seen from the blue side). [WPILib: coordinate system](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html) |
| **HUB** | The 2026 scoring target, a 47-inch-square tower with an opening on top. Each alliance has one. |
| **ID (identifier)** | A number or name that tells one item apart from the others, like a tag number or a motor's CAN ID. |
| **IP address** | A number that identifies a computer on a network, like a street address. `127.0.0.1` always means "this same computer". Our robot is at `10.95.20.2`. |
| **Jetson** | A small extra computer (made by the computer-chip company NVIDIA) that can run four extra cameras through PhotonVision. It is optional on our robot. |
| **JDK (Java Development Kit)** | The software that lets a computer build and run Java programs. The WPILib installer includes the right one (Java 17). |
| **Latency** | Delay. A camera with 30 milliseconds of latency reports what it saw 30 milliseconds ago. |
| **Limelight** | A smart camera made for FRC robots. Our robot has a **Limelight 3** (used to find the robot's position) and a **Limelight 4** (used to aim the shooter). [Limelight documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2) |
| **Localhost** | Another name for "this same computer" (`127.0.0.1`). |
| **Localization error** | How far the estimate is from the truth: *does the robot know where it is?* |
| **MegaTag2** | The method our Limelight 3 uses to calculate the robot's position from AprilTags. It needs the robot to tell it which way it is facing. [Limelight: MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2) |
| **Milliseconds (ms)** | Thousandths of a second. The robot program repeats its main loop every 20 milliseconds (50 times per second). |
| **Mount (camera mount)** | Where a camera is attached on the robot and which way it points. |
| **NetworkTables** | The system the robot uses to share live information (numbers, positions, settings) with dashboards and other programs over the network. The simulator's web page uses it too. [WPILib: NetworkTables](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html) |
| **Node.js** | A program that runs the simulator's web page on your laptop. |
| **Odometry** | Working out where the robot is by counting how far each wheel turned and reading the gyro. It drifts over time because wheels slip. |
| **OUTPOST** | The human-player station in one corner of each alliance's end of the field. |
| **Path** | A curved line on the field that the robot follows, saved as a file ending in `.path`. |
| **PathPlanner** | The free tool (and code library) our robot uses to create and follow paths and autos. The simulator's Plan tab edits the same files. [PathPlanner documentation](https://pathplanner.dev/home.html) |
| **PhotonVision** | Free camera software that can run on the Jetson to find AprilTags with four extra cameras. [PhotonVision documentation](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html) |
| **PID control (proportional–integral–derivative control)** | A common way to make a motor or robot reach a target smoothly: it pushes harder the farther away it is. [WPILib: introduction to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html) |
| **Pipeline** | A camera setting that tells the Limelight what to look for. Our Limelight 4 has four: Hub Blue, Hub Red, Trench Blue and Trench Red. |
| **Port** | A numbered "door" on a computer that a program listens on. The simulator's web page uses port 5173, and the robot program's NetworkTables uses port 5810. |
| **Pose** | A robot's position and heading together: *where it is and which way it faces*. |
| **roboRIO** | The robot's main computer, made by National Instruments. The robot program runs on it. |
| **Rotation target** | A point along a path where you tell the robot which way to face. |
| **Setpoint** | Where the path follower wants the robot to be *right now*. Shown as an orange outline. |
| **Swerve drive** | Our drivetrain type: each of the four wheels can steer by itself, so the robot can drive in any direction while facing any direction. |
| **TalonFX** | The motor controller (made by CTRE) built into our Kraken motors. |
| **Teleoperated ("teleop")** | The part of the match after autonomous, when drivers control the robot. |
| **TOWER** | The climbing structure built into each alliance wall. |
| **Tracking error** | How far the estimate is from the setpoint: *is the robot keeping up with the path?* |
| **TRENCH** | A structure next to each side wall that robots can drive under if they are short enough (22.25 inches of clearance). |
| **Truth** | Where the robot *really* is. Only the simulator can show this. Solid robot with a green outline. |
| **tx / ty** | What the Limelight 4 reports about the target: **tx** is how many degrees left or right of the center of the picture it is (positive means right), and **ty** is how many degrees up or down. |
| **USB (Universal Serial Bus)** | The common plug and cable used for keyboards, phones and memory sticks. The robot's main computer can also be reached over a USB cable. |
| **Waypoint** | A point the path passes through. Shown as a numbered circle. |
| **WPILib** | The official free software library and tools for FRC robots, including the simulation tools this project uses. [WPILib documentation](https://docs.wpilib.org/en/stable/) |

---

## 3. Before you start (one-time setup)

You only need to do this once per laptop.

1. **Install WPILib.** Download and run the WPILib installer for 2026 by following the [WPILib installation guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html). It installs Visual Studio Code (a code editor), the correct version of Java (the JDK), AdvantageScope and Elastic. The simulator also works with the 2025 installer if that is what your laptop already has.
2. **Install Node.js.** Download the "LTS" version (that stands for "Long-Term Support", meaning the stable version) from the [Node.js download page](https://nodejs.org/en/download) and install it with the default options.
3. **Get the code.** Ask a mentor for the code folder (`FRC-25-26-code-for-9520`), or download it with git (see [section 11](#save-your-work-with-git) for what git is).

The first time you start the simulator, it downloads some extra libraries from the internet, which can take a few minutes. After that it works without internet, which is handy in the pits at competitions.

---

## 4. Starting and stopping the simulator

The simulator has two halves that run at the same time:

- **The robot program**, running on your laptop as if it were the robot.
- **The web page** that shows the field and has all the buttons.

### Starting (Mac or Linux)

1. Open the **Terminal** app.
2. Go to the code folder by typing `cd ` (with a space), dragging the `FRC-25-26-code-for-9520` folder onto the Terminal window, and pressing **Enter**.
3. Type one of these and press **Enter**:
   - `./sim.sh` starts both halves **and** opens the WPILib simulation window (useful if you want to drive with a keyboard or a game controller).
   - `./sim.sh --headless` starts both halves **without** the extra window. "Headless" just means "no window". This is the usual choice, because everything you need for autos is on the web page.
4. Wait until the Terminal shows `Robot program startup complete`.
5. Open a web browser (Chrome works best) and go to **http://localhost:5173**.
6. In the top-right corner of the page you should see **● robot sim connected** in green.

### Starting (Windows, or without the script)

1. Open the code folder in WPILib Visual Studio Code, press **Ctrl+Shift+P**, and choose **WPILib: Simulate Robot Code**. ([WPILib: simulation introduction](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html))
2. In a separate terminal, go into the `sim-ui` folder and run `npm install` (first time only) and then `npm run dev`.
3. Open **http://localhost:5173**.

### Stopping

Click on the Terminal window and press **Ctrl+C** (hold the Control key and press C). This stops both halves.

### Messages you can ignore

While the simulator runs, the Terminal prints warnings such as *"Could not find any PhotonVision coprocessors"* or *"Joystick Button … not available"*. These are normal. They appear because there is no real Jetson computer or game controller plugged in. You only need to worry if you see `BUILD FAILED`, a long error that starts with `Exception`, or the message `Robot sim exited`.

---

## 5. A tour of the screen

At the very top of the page is the **top bar**:

- **9520 Autonomy Simulator — 2026 REBUILT** is the title. "REBUILT" is the name of the 2026 game.
- **Plan** and **Run** are the two tabs. Click one to switch.
  - The **Plan tab** is where you draw paths and build autos.
  - The **Run tab** is where you run them and study what happened.
  - Switching tabs does not lose your work: your last recorded run stays in the Run tab.
- **Field: AndyMark / Welded.** Official FRC fields are built by two different suppliers, and their AprilTags are in very slightly different places (about 1.5 centimeters apart). Our robot program uses the **AndyMark** layout. If you pick Welded, a small orange warning appears to remind you that the robot program does not match.
- **The connection light** shows **● robot sim connected** (green) when the web page can talk to the robot program, or **○ robot sim not running** when it cannot.

The big picture in the middle of both tabs is the **field**, drawn to scale and seen from above. The blue alliance wall is on the left and the red alliance wall is on the right. You will see:

- the two **HUBs** (grey squares in the middle of each half),
- the **BUMPs** (striped yellow ramps) and **TRENCHes** (dashed grey areas) next to each HUB,
- the **TOWERs** (grey blocks built into each alliance wall),
- the **DEPOT** and **OUTPOST** outlines,
- the **starting lines** (blue and red lines across the field) and the dashed **center line**,
- every **AprilTag**, drawn as a short black bar with a tick showing which way it faces, plus its number.

If you rest your mouse on any field part or tag, a small box tells you exactly what it is, its size, and where its position comes from. A `*` after **DEPOT** means its exact position along the wall is an estimate.

---

## 6. The Plan tab: drawing paths and building autos

The Plan tab is a path editor, similar to the official PathPlanner app. It edits the **real** PathPlanner files inside the robot code (in the folder `src/main/deploy/pathplanner`). The PathPlanner app can open the same files, and the robot follows exactly what you save here.

### 6.1 The file list (left side)

- **Paths** lists every path. Click a name to open it. The **＋** button next to the heading creates a new path. It asks you for a name and then draws a simple starting curve that you can reshape.
- **Autos** lists every auto. Click to open, or press **＋** to create a new, empty auto.
- **⚙ Robot settings** opens the robot's measurements (see [6.7](#67-robot-settings)).
- At the bottom you can see which folder the files come from.

### 6.2 The toolbar (top, when a path is open)

- **The file name** (for example **Test path**) is shown in bold. A dot **•** after it means you have changes that are not saved yet.
- **Save** writes your changes to the file. It is grey when there is nothing to save. Keyboard shortcut: **Ctrl+S** (Windows) or **Cmd+S** (Mac).
- **↶ (Undo)** takes back your last change. Shortcut: **Ctrl+Z** / **Cmd+Z**.
- **↷ (Redo)** puts back a change you undid. Shortcut: **Shift+Ctrl+Z** / **Shift+Cmd+Z**.
- **＋ Waypoint** turns on "add mode". While it is on, every click on the field adds a new waypoint at the end of the path. A blue banner reminds you. Press **Esc** or click the button again to stop.
- **◷ Preview** plays a "ghost" robot along the path and shows a speed chart underneath the field.
  - While the robot simulation is running, the timing is **exact**: it is calculated by the same PathPlanner code the real robot uses, with the robot's real measurements.
  - Without the simulation running, you get an **approximate** preview, and the chart says so.
  - The chart shows the total time, for example "16.32 s total".
- **✕** (next to Preview) closes the preview.
- **footprint** (checkbox) shows or hides faint outlines of the robot along the path, so you can see how much room it takes up.
- **▶ Run this** saves the path, switches to the Run tab and plays it immediately. It only works when the robot simulation is running.
- **Duplicate** makes a copy of the file under a new name.
- **Rename** changes the file name. Be careful: if an auto uses this path, you must update the auto to the new name, or the auto will not find it.
- **Delete** removes the file after asking you to confirm. A backup copy is kept in the `sim-ui/.sim-backup` folder, so a mentor can recover it.

### 6.3 Editing the path on the field

- **Waypoints** are the numbered circles. Waypoint **0** (green) is the start and the last one (orange) is the end. **Drag** a waypoint to move it. Click a waypoint to select it and show its **Bézier handles**.
- **Bézier handles** are the small filled dots on lines coming out of the selected waypoint. Drag one to change how the path curves through that waypoint. Normally the two handles stay in a straight line so the path stays smooth. Hold **Shift** while dragging to move one handle by itself and make a sharp corner.
- **Start and end robots** are the purple robot outlines at the start and end. The small circle in front of each one is a **heading handle**: drag it around to choose which way the robot faces at the start or the end. It snaps to 5-degree steps. Hold **Shift** for any angle.
- **Rotation targets** are dashed robot outlines along the path. Drag the dot in the middle to slide one along the path, or drag the circle in front of it to change which way the robot faces there.
- **Event markers** are orange diamonds ◆ with a label. Drag them along the path to choose where a named command starts.
- **Red dots** appear where the robot's bumpers would hit the HUB, a TOWER or a wall. The Warnings panel on the right says which part and near which waypoint. Driving over a BUMP or under a TRENCH is allowed, so those do not make red dots.

### 6.4 The side panels (right side, when a path is open)

- **Warnings** appears only when there is a problem: the bumpers would hit something, or a marker uses a named command that the robot code does not know.
- **Waypoints** is a table with the x and y position (in meters) of every waypoint. You can type exact numbers. Click a row to select that waypoint.
  - **Insert after** adds a new waypoint halfway between the selected one and the next.
  - **Delete** removes the selected waypoint. A path always keeps at least two.
- **Start & end** sets:
  - **Start heading** and **End heading**: which way the robot faces, in degrees.
  - **Start velocity** and **End velocity**: how fast the robot is moving at the start and end. Use 0 to start and stop at rest.
- **Constraints** are the speed limits for this path:
  - **Max velocity** (meters per second): our robot's top speed is about 4.44.
  - **Max accel** (meters per second, per second): how quickly it may speed up or slow down.
  - **Max turn rate** (degrees per second): how fast it may spin.
  - **Max turn accel** (degrees per second, per second): how quickly it may start and stop spinning.
  - Lower limits make the robot slower but more accurate.
- **Rotation targets** has **＋ Add**, which adds one. Each target has a slider ("at waypoint", see below), a **heading** box, and **✕** to remove it.
- **Event markers** has **＋ Add**, which adds one. Each marker has:
  - a name box,
  - an "at waypoint" slider,
  - a **runs** menu where you pick the named command,
  - **✕** to remove it.

**What does "at waypoint 1.5" mean?** Positions along a path are measured in waypoints: 0 is the first waypoint, 1 is the second, and 1.5 is halfway between the second and third. This is how PathPlanner stores positions.

### 6.5 Named commands you can use

These are the actions the robot code makes available for your autos:

| Name | What it does | Where you can use it |
|---|---|---|
| **SpinUp** | Starts the shooter flywheel spinning (the feeder stays off). | Event marker or auto step |
| **Shoot** | Spins the flywheel up (waiting up to 1.5 seconds), then feeds balls for 1 second. | Event marker or auto step |
| **ShootAll** | Same as Shoot, but feeds for 4 seconds to empty the robot. | Event marker or auto step |
| **StopShooter** | Stops the shooter and the feeder. | Event marker or auto step |
| **IntakeOn** / **IntakeOff** | Turns the intake roller on or off. | Event marker or auto step |
| **ArmDown** / **ArmUp** | Moves the intake arm down or up for 0.6 seconds. | Event marker or auto step |
| **AutoAim** | Turns the robot in place until the Limelight 4 is pointed at the HUB (gives up after 2 seconds). | **Auto step only** |
| **AimAndShoot** | AutoAim, then ShootAll. | **Auto step only** |
| **WaitShoot1**, **Waitshootall** | The same as Shoot and ShootAll. These names are used by the example `Test path`. | Event marker or auto step |

**Why can't AutoAim be an event marker?** An event marker runs *while the robot keeps driving the path*. AutoAim needs to control the wheels to turn the robot, and only one thing can control the wheels at a time, so it would stop the path. Put AutoAim and AimAndShoot as their own steps in an auto instead. You can read more in the [PathPlanner named commands documentation](https://pathplanner.dev/pplib-named-commands.html).

### 6.6 Building an auto

When you open an auto (a `.auto` file), the right side shows the **auto builder**.

- **Reset odometry to the first path's start** (checkbox). When this is on, the robot program assumes the robot starts exactly at the start of the first path. Leave it on unless you know why you need it off.
- **Steps** is the list of everything the auto does. The top item is a **group** that holds the other steps. Each row has:
  - a **type** menu (see the groups and steps below),
  - a second menu or box for the details (which path, which named command, or how many seconds to wait),
  - **↑** and **↓** to move the step earlier or later,
  - **✕** to remove it.

**Group types** (a group holds several steps):

- **sequential**: the steps run one after another. This is the most common.
- **parallel**: all the steps start at the same time, and the group ends when **all** of them have finished.
- **race**: all the steps start at the same time, and the group ends as soon as **any one** of them finishes.
- **deadline**: all the steps start at the same time, and the group ends when the **first step in the list** finishes.

**Step types:**

- **path**: drive one of your paths.
- **named**: run a named command.
- **wait**: do nothing for a number of seconds.

Under each group are buttons to add things to it: **＋ path**, **＋ named**, **＋ wait** and **＋ group**.

While an auto is open, the field shows all the paths it uses, in order and in different colors, with the starting robot position. The Warnings panel tells you if a path is missing or a named command does not exist.

### 6.7 Robot settings

**⚙ Robot settings** edits the file `settings.json`, which tells PathPlanner how heavy and how fast the robot is. PathPlanner uses it to plan paths the robot can actually drive. Settings marked with **\*** are still guesses and must be measured on the real robot:

- **Length with bumpers** and **Width with bumpers** (meters)
- **Mass**: the weight of the robot with its battery and bumpers, in kilograms
- **Moment of inertia**: how hard it is to spin the robot
- **Wheel friction**: how grippy the wheels are on carpet
- **Drive motor** type and **Drive current limit** (the most electrical current, in amperes, the drive motors may use)

The other values come from the robot's drivetrain setup and normally do not need changing:

- **Wheel radius** is the size of the drive wheels, in meters.
- **Drive gearing** is how many times the motor turns for one turn of the wheel.
- **Max drive speed** is the robot's top speed, in meters per second.
- **Default max velocity (new paths)** and **Default max accel (new paths)** are the starting speed limits given to every new path you create.

The **Checks** box shows the size in inches and reminds you of the game rules. It warns you if the size here does not match the size the simulator draws.

---

## 7. The Run tab: running and understanding what happened

The Run tab is where you play an auto through the real robot program and study the result.

### 7.1 The run bar (top row)

- **The status badge** on the far left shows what the robot is doing: **DISABLED**, **AUTO**, **TELEOP**, **PAUSED**, or **NO ROBOT SIM** if the robot program is not running.
- **Auto / Path / Hand-coded** chooses what kind of thing to run:
  - **Auto** runs one of your `.auto` files.
  - **Path** runs a single `.path` file by itself.
  - **Hand-coded** runs one of the autos written directly in Java by the programmers. These are "Shoot All (Stationary)", "Do Nothing" and "Drive Forward 5s".
- **The name menu** picks which one to run.
- **Blue / Red** chooses the alliance. On red, the robot program automatically mirrors the path to the red side, just as it does in a real match.
- **▶ Play** runs it for real. Here is what happens when you press it:
  1. The simulator reads your files fresh, so any change you just saved is used.
  2. It puts the robot on the start position. If you set a placement error (see [7.6](#76-the-side-panels)), the *true* robot is placed slightly off while the robot program still believes it is exactly on the start.
  3. It turns on autonomous mode. The robot program starts the auto by itself, exactly like in a match.
  4. After 20 seconds it disables the robot, like the end of the autonomous period.
- **⟲ Reset** stops everything and puts the robot back on the start position, ready to play again.
- **■ Stop** stops everything and disables the robot immediately.
- **⏸ Pause** freezes time for the whole simulation: the program, the motors and all the timers stop together. While paused, two other buttons appear:
  - **▶ Resume** continues.
  - **⏭ 20 ms** moves forward by exactly one robot loop (20 milliseconds), so you can watch a moment frame by frame.
- **The clock**, for example **t = 6.42 / 20 s**, shows how far into the 20-second autonomous period you are. It shows a purple **REPLAY** tag when you are looking at a recorded moment instead of the live robot.

### 7.2 The tool bar (second row)

- **Place robot** puts the robot somewhere else. Click it, then click on the field where you want the robot, and drag in the direction you want it to face before letting go. The heading snaps to 15-degree steps; hold **Shift** for any angle. This moves both the true robot and the estimate.
- **Kidnap** moves **only the true robot**, and the robot program is not told. This is like someone bumping the robot during a match. Click it, then click where the robot "really" is now. Watch whether the cameras notice and pull the estimate over.
- **Sync truth** snaps the true robot back onto the estimate and clears all the built-up drift.
- **Test: square** drives the robot around a 2-meter square at 1.5 meters per second. It is a quick way to see drift without writing an auto.
- **Test: spin** spins the robot in place for two full turns. It shows how the gyro's small errors add up when turning.
- **● Live** appears while you are looking at a recorded moment. It returns to the live robot.
- **Clear traces** erases the lines the robots have drawn behind them.

### 7.3 What the colors on the field mean

| You see | It means |
|---|---|
| **Purple line** | The **plan**: your path. It is thicker while the robot is following that path. |
| **Purple robot outline** | The planned start position (before you press Play). |
| **Orange robot outline** | The **setpoint**: where the path follower wants the robot right now. |
| **Dashed blue outline and dashed blue line** | The **estimate**: where the robot program thinks it is, and where it has been. |
| **Solid robot with green outline, and green line** | The **truth**: where the robot really is, and where it has really been. |
| **Small orange arrows on the wheels** | Which way each wheel is pointing and how fast it is turning. |
| **Green, pink and blue wedges** | What each camera can see: green for the Limelight 3, pink for the Limelight 4, and blue for the four PhotonVision cameras. |
| **Dashed lines from a camera to tags** | The AprilTags that camera can see right now. |
| **Hollow green circle** | The robot position the Limelight 3 reported most recently. |
| **Small green dots** | Camera measurements that the robot program actually **used** to correct its estimate. They fade after a couple of seconds. |
| **Green arrow into a HUB** | A FUEL shot that **scored**. |
| **Red dashed arrow with a circle** | A FUEL shot that **missed**, ending where the ball would land. |

### 7.4 The chart under the field

When you press Play, the chart records the whole 20-second run with two lines:

- **Tracking error** (orange): how far the estimate is from the setpoint. If this is big, the robot is **not keeping up with the path**. The path may be too fast or too sharp, or the path-following settings may need tuning.
- **Localization error** (blue): how far the estimate is from the truth. If this is big, the robot **does not know where it is**, because of drift, being placed in the wrong spot, or bad camera information.

**Click or drag on the chart** to jump to any moment of the recorded run. The field then shows the robots exactly as they were at that moment (the clock shows **REPLAY**). Press **● Live** to go back.

When nothing has been played yet (for example after a test drive), the chart instead shows the live **position error** and **heading error** for the last 30 seconds.

### 7.5 The Layers panel

Each checkbox shows or hides one kind of drawing, so you can focus on what matters:

- **Plan — the path file**
- **Setpoint — where it should be now**
- **Estimate — what the code believes**
- **Truth — where it really is**
- **Camera views + sightlines**
- **Vision measurements used**
- **Fuel shots**
- **Traces** (the lines behind the robots)
- **AprilTag IDs** (the tag numbers)

### 7.6 The side panels

**Last run** (it says **Run** before your first Play) summarizes the most recent run:

- **Max tracking**: the biggest tracking error during the run.
- **Max localization**: the biggest localization error during the run.
- **Final localization**: the localization error at the end.
- **Auto finished**: the second when your auto completed, or "not in time" if it was still running when the 20 seconds ended.
- **Fuel scored** and **Fuel missed**.

The numbers turn green when they are good, orange when they are so-so, and red when they need attention.

**Cameras** has one card for each camera:

- **Limelight 3 — MegaTag2 pose** works out the robot's position from AprilTags.
- **Limelight 4 — shooter aim** looks for the HUB and tells the aiming code how far off it is.
- **PhotonVision ×4 (Jetson)** is the set of four extra cameras. It is **off** by default because the Jetson is optional on our robot.

On each card:

- The **checkbox** next to the camera's name turns that camera on or off. A camera that is off sends nothing, and after a moment the robot program reports it as offline, just like an unplugged camera.
- The grey text under the name tells you which tags it currently sees.
- **Noise** makes the camera's measurements more or less jittery. 1× is normal, 0 is perfect, and 5× is very noisy.
- **Latency** is how old each picture is when it arrives, in milliseconds.
- **Dropped frames** (Limelights only) is the percentage of pictures that get lost.
- **Mount error X / Y / Yaw** pretends the camera is bolted on slightly differently from what the robot program believes: shifted forward or back (X), left or right (Y), or turned (Yaw). This creates a steady error you can learn to recognize. **Zero** puts the mount back to perfect.
- On the Limelight 4 card, a small black **camera picture** shows what that camera sees: the crosshair in the middle, a pink box where the target is, and the **tx** and **ty** numbers. The aiming code turns the robot until tx is close to 0.
- **Reset cameras** puts every camera back to its default settings.

**Fuel** shows how many balls are **in robot**, **scored** and **missed**, and the reason for the last shot (for example "missed — aim off"). In this simulator a ball scores when all three of these are true at the moment it leaves:

1. The robot is really pointing within the **aim tolerance** of its own HUB.
2. It is between 1.5 meters and the **max distance** from the HUB.
3. The flywheel is spinning at least 90 percent of its target speed.

There is no ball flight; it is a simple rule. The panel has these controls:

- **Preload**: how many balls the robot starts with. The rules allow up to 8.
- **Aim tolerance**: how precisely the robot must aim.
- **Max distance**: the farthest the robot can be from the HUB and still score.
- **Reload fuel**: refills the robot. Fuel is also refilled automatically every time you press Play or Reset.

**Mechanisms** shows bars for the **Flywheel** (actual speed and target speed; a ✓ means it is up to speed), the **Feeder**, the **Intake**, the **Intake arm**, and the left and right **Climbers**. These move because the real robot program is sending commands to the simulated motors.

**Placement error** is how far from the planned start the robot is really set down, as happens when students place the robot by hand before a match. The robot program believes the robot is exactly on the start; only the true robot is moved. The controls are:

- **X offset** and **Y offset** (centimeters).
- **Heading offset** (degrees).
- **Perfect**: sets everything back to zero.
- **Random (±10 cm)**: picks a random small error.

**Localization now** shows the current position and heading error between the estimate and the truth.

**Drift** controls how imperfect the robot's wheels and gyro are.

- **Ideal**: perfect wheels and a perfect gyro, so there is no drift.
- **Well-tuned**: a good robot with fresh wheels. Expect roughly 10 to 20 centimeters of error after a 20-second auto with no cameras.
- **Worn wheels**: worn tread or dusty carpet. Errors roughly double.
- **Rough match**: lots of slipping, like a match with hard pushing.
- **Wheel scale error**: the wheels think they traveled a little farther than they did.
- **Random slip**: small random slips that add up as the robot drives.
- **Hard-accel threshold** and **Hard-accel slip ×**: above this acceleration the wheels lose grip, and slip is multiplied by this amount.
- **Gyro bias drift**: the gyro's slow creep while the robot is on.
- **Gyro scale error**: the gyro's small error each time the robot turns, which shows up when spinning.
- **Random seed**: a number that controls the "random" parts. If two runs use the same seed and the same settings, they slip in exactly the same way, so you can compare them fairly.
- The line at the bottom estimates how far off the robot would be after a 10-meter straight drive with these settings.

**Robot** shows the robot size the simulator is using and reminds you that it is a placeholder until the robot is measured.

---

## 8. Practice scenarios

Each scenario lists what to do, what you should see, and what it teaches. Start the simulator first (section 4).

### Scenario 1 — Your first path

1. Go to the **Plan** tab and press **＋** next to **Paths**. Name it "My first path".
2. Drag the end waypoint somewhere in the blue half of the field. Use **＋ Waypoint** to add one or two more points, and drag the handles to make a smooth curve that avoids the HUB.
3. Drag the heading handle of the end robot so the robot ends facing the HUB.
4. Press **◷ Preview** and watch the ghost robot. Note the total time.
5. Press **▶ Run this**.

**You should see** the robot follow your purple line in the Run tab, with the orange setpoint just ahead of it.
**It teaches** the whole loop: draw, check, run.

### Scenario 2 — A perfect robot versus a real robot

1. In the Run tab, turn **off** all cameras.
2. Choose **Drift → Ideal**, pick **Path → Test path**, and press **▶ Play**. Look at **Max localization** in Last run. It should be about 0 centimeters.
3. Choose **Drift → Well-tuned** and press **▶ Play** again.

**You should see** the green truth line slowly separate from the dashed blue estimate, ending about 10 to 20 centimeters apart.
**It teaches** that even a good robot slowly loses track of where it is when it only counts wheel turns.

### Scenario 3 — Setting the robot down in the wrong spot

1. Keep cameras off. In **Placement error**, set **X offset** to 20 centimeters.
2. Play a path.

**You should see** the whole green path shifted 20 centimeters from the plan, even though the robot program thinks it is perfectly on track (tracking error stays small, localization error is about 20 centimeters).
**It teaches** that the robot follows its *belief* perfectly. If it was placed wrong, it drives the right shape in the wrong place. This is why careful robot placement before a match matters, and why cameras are useful.

### Scenario 4 — The robot gets bumped

1. Turn the **Limelight 3** camera **on**. Place the robot about 2.5 meters in front of the blue HUB, facing it, so the camera sees tags 25 and 26.
2. Press **Kidnap** and click about 70 centimeters away.
3. Watch **Localization now**.
4. Turn the Limelight 3 **off**, and kidnap again.

**You should see:**
- With the camera on, the error drops from about 70 centimeters to under 2 centimeters in about half a second, and green dots appear where the camera measurements were used.
- With it off, the error simply stays.

**It teaches** how cameras fix the robot's belief.

### Scenario 5 — Why the camera cannot fix a wrong heading

1. Turn the Limelight 3 on. In **Placement error**, set **Heading offset** to 2 degrees and the X and Y offsets to 0.
2. Play a path that passes in front of the blue HUB.

**You should see** a localization error of about 9 to 13 centimeters that does **not** go away near the tags, even though the camera can see them.
**It teaches** how MegaTag2 works. It works out position *using the heading the robot program gives it*. If that heading is wrong, the position it calculates is wrong too, and it cannot correct the heading. A heading error of 2 degrees at 2.5 meters becomes about 9 centimeters of position error. On the real robot, this is why the gyro must be correct at the start of the match.

### Scenario 6 — A camera bolted on in the wrong place

1. Turn the Limelight 3 on and the others off. Set its **Mount error X** to 10 centimeters.
2. Drive the **Test: square** in view of the tags.

**You should see** the estimate settle about 10 centimeters away from the truth, consistently.
**It teaches** that if the camera's position on the robot is measured wrong, every measurement is wrong by the same amount. Measure camera mounts carefully.

### Scenario 7 — A path that is too fast or too sharp

1. Copy a path with **Duplicate**. In **Constraints**, raise **Max velocity** to 4.4 and **Max accel** to 6.
2. Add a sharp turn by Shift-dragging a handle.
3. Run it, and look at the **Tracking error** line in the chart.

**You should see** the tracking error jump at the sharp turn and at the start, because the robot cannot keep up.
**It teaches** that tracking error is a path or tuning problem. Lowering the constraints, or smoothing the curve, makes the robot follow the plan much more closely.

### Scenario 8 — A full scoring auto

1. Make a path that ends between about 2 and 4 meters from your HUB, facing it, so the Limelight 4 can see the HUB tags.
2. Create a new auto. In **Steps**, add **＋ path** (your path) and then **＋ named** with **AimAndShoot**. Save it.
3. Make sure the **Fuel** preload is 8. Press **▶ Run this**.

**You should see:**
- The robot drives the path.
- It turns until the pink box in the Limelight 4 picture is centered.
- The flywheel bar fills up, and green shot arrows go into the HUB.
- **Fuel scored** counts up.
- If a ball misses, the Fuel panel tells you why ("aim off", "too far", "flywheel too slow").

**It teaches** how driving, aiming and shooting work together.

You can also try the hand-coded **Shoot All (Stationary)** auto. Place the robot facing roughly toward the HUB from about 3 meters away, choose **Hand-coded**, and press **▶ Play**.

### Scenario 9 — Playing as the red alliance

1. Pick any auto and choose **Red** in the run bar. Press **▶ Play**.

**You should see** the same auto mirrored to the red side of the field, rotated 180 degrees around the field center.
**It teaches** that you only need to draw autos once, from the blue side. The robot program mirrors them automatically when the Driver Station says the robot is on the red alliance.

### Scenario 10 — Freeze a moment and step through it

1. During a run, press **⏸ Pause**.
2. Press **⏭ 20 ms** a few times and watch the setpoint, estimate and truth move one robot loop at a time.
3. Press **▶ Resume**.

**It teaches** what the robot is doing 50 times per second.

### Scenario 11 — Comparing two runs fairly

1. Set **Random seed** to a number you will remember (for example 9520) and pick **Well-tuned**.
2. Play your auto and write down **Max localization**.
3. Change exactly one thing (for example turn a camera on, or lower the max velocity), keep the same seed, and play again.

**It teaches** how to test fairly: with the same seed, the random slipping is identical, so any difference comes from the one thing you changed.

---

## 9. Watching in AdvantageScope and Elastic

You can watch the simulator with the same tools the team uses on the real robot. They can be open at the same time as the simulator's web page.

**AdvantageScope** ([live connection documentation](https://docs.advantagescope.org/overview/live-sources/), [2D field documentation](https://docs.advantagescope.org/tab-reference/2d-field/)):

1. Start the simulator.
2. In AdvantageScope, choose **File → Connect to Simulator → NetworkTables 4**. It connects to `127.0.0.1` by itself; you do not type an address.
3. In settings (**AdvantageScope → Settings…** on Mac), set **Live Mode** to **Logging** so you can scroll back over a whole run.
4. Add a **2D Field** tab, set the field to **2026 REBUILT**, and drag these entries from the left side into **Poses**:
   - `DriveState/Pose` as the robot (this is the estimate).
   - `/Sim/Truth` as a ghost (the truth).
   - `/Sim/Setpoint` as a second ghost.
   - `/Sim/ActivePath` as a trajectory (the plan).
5. AdvantageScope will warn that the last three use an older "number array" format. Choose **Pose2d** `[x, y, rotation]` with **radians**.
6. Add a **Swerve** tab with `DriveState/ModuleStates` and `DriveState/ModuleTargets` to see the wheels.

If the 2026 field is not in the list, install the 2026 version of AdvantageScope (it comes with the WPILib 2026 installer).

**Elastic** ([Elastic documentation](https://frc-elastic.gitbook.io/docs)): connect it to `localhost` (the simulator) instead of team 9520, and you will see the same **Auto Chooser**, **Target Mode** and status lights the drivers see at competition.

| Where the robot program runs | Address to use |
|---|---|
| Simulator on your own laptop | `127.0.0.1` ("Connect to Simulator" fills it in) |
| Simulator on a teammate's laptop | That laptop's IP address |
| The real robot | `10.95.20.2` (or `172.22.11.2` over a USB cable) |

---

## 10. Switching between the simulator and the real robot

**The most important fact:** the simulator runs the **same robot program** that goes on the real robot. You do not change any code to switch between them.

The parts that only exist for the simulator (the true robot, drift, simulated cameras and motors, and the web page connection) live in the folder `src/main/java/frc/robot/sim`. The robot program only starts them when it detects it is running on a laptop. On the real robot they never run.

### Running in the simulator

Start it with `./sim.sh` (section 4) and use the web page.

### Running on the real robot

1. **Connect to the robot.** Turn the robot on and connect your laptop to the robot's Wi-Fi network (it is named after team 9520), or plug in a USB or Ethernet cable.
2. **Deploy the program** if it has changed (see [section 11](#11-putting-your-paths-and-autos-on-the-real-robot)).
3. **Open the Driver Station** on the Windows driver laptop. It should show green lights for "Communications" and "Robot Code". ([WPILib: Driver Station](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html))
4. **Open Elastic** (and AdvantageScope if you like) and connect them to team 9520 or address `10.95.20.2`.
5. **Pick your auto** in the **Auto Chooser** on Elastic.
6. **Choose the alliance** in the Driver Station (this decides whether paths are mirrored to red).
7. Set the Driver Station to **Autonomous** and press **Enable**. Always have someone ready to press **Disable**. In the Driver Station the **Enter** key disables the robot. The **space bar** is the emergency stop: it stops the robot instantly, but the robot must then be restarted before it can run again.

### What is different on the real robot

- **There is no "truth" view.** You only see the estimate on the dashboards. Your eyes on the real field are the truth. A good habit is to mark where the robot should end on the carpet with tape, and compare.
- **Real drift and real cameras.** The real robot slips and its cameras see real lighting and real tag positions. The simulator's numbers are educated guesses, so expect some differences.
- **Real mechanisms.** The real shooter, intake and climbers can jam, run out of battery, or be geared differently from the simulator's guesses.

### Safety checklist before the first real test of new code

These changes were made while building the simulator and **must be checked on the real robot** before a match:

1. **Put the robot on blocks** (wheels off the ground) the first time you enable it with new code.
2. **Check the joystick directions.** Pushing the left stick forward should drive forward, pushing it left should drive left, and pushing the right stick right should turn right (clockwise). If the robot drives sideways, the wheels' zero positions are 90 degrees off: ask a mentor to recalibrate them in CTRE Tuner X ([CTRE swerve documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)) instead of changing the code.
3. **Check auto-aim (button B, or the AutoAim command).** It should turn toward the HUB smoothly without overshooting back and forth. It may need tuning (the numbers `kP` and `MIN_ROT_RAD_S` in `AimController.java`).
4. **Check the shooter (button Y).** It should spin up first and then feed. If the flywheel never reaches speed, the Shoot commands feed anyway after 1.5 seconds.
5. **Check the Limelight 3 position estimate** while the robot sits still in front of tags. The robot program now ignores the Limelight 3 while the robot spins very fast (more than 360 degrees per second).

---

## 11. Putting your paths and autos on the real robot

Your paths and autos are already in the right place: the Plan tab saves them directly into the robot code, in the folder `src/main/deploy/pathplanner` (paths in `paths/`, autos in `autos/`). Everything in the `deploy` folder is copied onto the robot automatically when you deploy.

### Step 1 — Measure the robot (once, and whenever it changes)

PathPlanner plans paths using the robot's measurements, so wrong numbers mean paths that are too fast or too slow for the real robot. With a mentor:

- Weigh the robot with its battery and bumpers, and measure its length and width with bumpers.
- Enter them in the Plan tab under **⚙ Robot settings** (this updates `settings.json`).
- Also update `SimRobotConstants.java` in the `sim` folder, so the simulator draws the same robot. The Robot settings panel warns you if the two do not match.
- Measure where each camera is mounted (forward, sideways, height, and tilt) and update the camera settings: the Limelight web page for the Limelights, and `SimRobotConstants.java` for the simulator.

### Step 2 — Deploy the program to the robot

1. Connect your laptop to the robot (its Wi-Fi network, a USB cable or an Ethernet cable).
2. In WPILib Visual Studio Code, press **Ctrl+Shift+P** (Cmd+Shift+P on Mac) and choose **WPILib: Deploy Robot Code**. Or, in a terminal inside the robot project folder, run `./gradlew deploy`. ([WPILib: deploying robot code](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html))
3. Wait for **BUILD SUCCESSFUL**. The robot program restarts by itself with your new paths and autos.

### Step 3 — Pick your auto

In Elastic, open the **Auto Chooser** menu. Every `.auto` file appears with **PP:** (short for PathPlanner) in front of its name (for example **PP: My Auto**). The Java autos appear under their own names (for example **Shoot All (Stationary)**, which is the default). Pick yours before the match starts.

### Step 4 — Test carefully

1. **Start slow.** For the first real run, lower the path's **Max velocity** and **Max accel** in the Plan tab, deploy, and run it in an open space. Raise the limits step by step once it behaves.
2. **Check the start position.** Place the robot exactly where the path starts, facing the right direction. Remember scenarios 3 and 5: a few centimeters or degrees off at the start shows up all through the auto.
3. **Check the alliance.** Set the alliance in the Driver Station before enabling. The robot program mirrors paths to red automatically.
4. **Check the field type.** At an event, ask whether the field is an AndyMark or Welded build. Our robot program uses the AndyMark tag layout.
5. **Compare with the simulator.** If the real robot ends far from where the simulator predicted, use AdvantageScope logs to see whether it was tracking error (path too fast) or localization error (robot did not know where it was).

### Save your work with git

**Git** is a tool that keeps a history of every change to the code, so nothing gets lost and teammates can share work. ([About version control](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control))

- A **commit** is a saved snapshot of your changes with a short message describing them.
- A **branch** is a separate line of work (this project's simulator work is on the branch called `coach`).
- **Pushing** uploads your commits to GitHub so others can get them. You need permission on the team's GitHub repository to push; ask a mentor if it says "permission denied".

Ask a mentor before pushing changes that affect the real robot.

---

## 12. Troubleshooting and common questions

**The page says "○ robot sim not running".**
The robot program half is not running. Look at the Terminal for errors. If you started it with `./sim.sh`, make sure it printed `Robot program startup complete`. Then refresh the web page.

**The Terminal says "Java 17 not found".**
Install WPILib (section 3). The script automatically uses the Java that comes with any WPILib installation.

**The Terminal says "Port 5173 is already in use" (or port 5810).**
Another copy of the simulator is still running. Stop it with **Ctrl+C** in its Terminal window. If you cannot find it, run this command in a Terminal and start again:
`pkill -f frc.robot.Main; pkill -f simulateJava; pkill -f vite`

**I pressed ▶ Play while an auto was already running and the new one did not start.**
This is a known problem that has not been fixed yet. Press **■ Stop** first, then **▶ Play**.

**My new auto is not in the menu.**
Make sure you pressed **Save** in the Plan tab. The Run tab's menu updates within about a second. On the real robot, new autos appear in the Auto Chooser after you deploy (the robot program restarts).

**The preview says "approximate".**
The robot simulation is not running. Start it to get exact timing.

**The robot drives through the HUB in the simulator.**
The simulator does not stop the robot at walls or field parts yet (that is planned). Look for red dots in the Plan tab, which show where the bumpers would hit, and fix the path.

**Why is the localization error a little above zero with Ideal drift?**
If cameras are on, their small random noise nudges the estimate by a centimeter or two. Turn the cameras off and it goes back to exactly zero.

**With cameras on and Well-tuned drift, the error sometimes gets bigger near the HUB.**
The Limelight 3 is less accurate while the robot is turning, and the robot program currently trusts every Limelight 3 measurement the same amount, no matter how far away or how fast the robot is moving. Making that trust depend on distance and speed is a great improvement to try on the robot code.

**Where are the technical details?**
In [docs/simulator.md](simulator.md). The list of recent changes is in [CHANGELOG.md](../CHANGELOG.md).

---

## 13. Where to learn more

- [WPILib documentation (home)](https://docs.wpilib.org/en/stable/): the official guide to FRC programming.
- [WPILib: installing the tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- [WPILib: robot simulation introduction](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html) and [the simulation window](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html)
- [WPILib: deploying robot code](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html)
- [WPILib: Driver Station](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html)
- [WPILib: NetworkTables](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html)
- [WPILib: field coordinate system](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
- [WPILib: pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html), which explains how the robot blends wheel counting with camera measurements.
- [WPILib: AprilTags](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)
- [WPILib: introduction to PID control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [WPILib: command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html), the style our robot code uses.
- [PathPlanner documentation](https://pathplanner.dev/home.html) and [named commands](https://pathplanner.dev/pplib-named-commands.html)
- [Limelight: MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
- [PhotonVision: simulation](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html)
- [CTRE Phoenix 6: swerve setup in Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)
- [AdvantageScope: live connection](https://docs.advantagescope.org/overview/live-sources/) and [2D field](https://docs.advantagescope.org/tab-reference/2d-field/)
- [Elastic documentation](https://frc-elastic.gitbook.io/docs)
- [2026 FRC Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)
- [Node.js download](https://nodejs.org/en/download)
- [About version control (git)](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control)
