# FRC2026
Welcome to Ridge Robotics' repository for the FRC 2026 season!

## Controls
Layout:
```
Back  Start

  Y
X   B
  A
```
### Driver
| Button              | Action                    |
|---------------------|---------------------------|
| Start               | Reset gyro/yaw            |
| Left Stick          | Drive (field-oriented)    |
| Right Stick         | Rotate (robot-oriented)   |
| Right Stick (press) | Lock yaw towards velocity |
| L Bumper            | Slow mode                 |
| L Trigger           | Intake                    |
| R Trigger           | Fast mode                 |
| X                   | Lock mode                 |
| Y                   | Face towards hub          |
| B                   | Wind up & launch          |
### Operator
| Button             | Action                                   |
|--------------------|------------------------------------------|
| L Trigger          | Disable driver fuel controls             |
| L Bumper + Start   | Auto test                                |
| R Bumper + Start   | Straighten wheels                        |
| L Bumper           | Eject                                    |
| R Bumper           | Wind up                                  |
| L Trigger + Start  | Toggle Back Camera                       |
| R Trigger + Start  | Toggle Front Camera                      |
| R Trigger          | Wind up & shoot                          |
| Both Triggers      | Shoot (no wind up)                       |
| A                  | Intake                                   |
| B                  | Unjam                                    |
| Back + other       | Enable MAX POWER for fuel                |
| L Stick Up/Down    | Spin with absolute rotation              |
| L Stick Left/Right | Increase/decrease manual launch velocity |
| L Stick Pressed    | Enable manual launch                     |
| R Stick Left/Right | Increase/decrease calculator velocity    |
| R Stick Pressed    | Toggle force normal drive mode           |
| Y                  | Climb                                    |
| Y + Start (RB)     | Climb (ignore limit)                     |
| Y + Back  (LB)     | Climb (fast)                             |
| X                  | Lower climber                            |
| X + Start (RB)     | Lower climber (ignore limit)             |
| X + Back  (LB)     | Lower climber (fast)                     |

| Button             | Action                                   |
|--------------------|------------------------------------------|
| Back + other       | Enable MAX POWER for fuel                |
| L Trigger          | Disable driver fuel controls             |
| Both Triggers      | Shoot (no wind up)                       |
| B                  | Un-shoot                                 |
| L Bumper           | Eject                                    |
| R Trigger          | Wind up & shoot                          |
| A                  | Intake                                   |
| L Stick Pressed    | Enable manual launch                     |
| R Stick Pressed    | Toggle force normal drive mode           |
| L Stick Left/Right | Increase/decrease manual launch velocity |


## LED Signals
Note: Ordered by priority

| State                | LED Pattern                       |
|----------------------|-----------------------------------|
| E-Stopped            | Red (strobe)                      |
| Disabled             | Green/White                       |
| X seconds left       | White (strobe)                    |
| Bump Lock Overridden | Cyan (strobe)                     |
| Bump Lock            | Cyan                              |
| Winding Up           | Red/Green (depending on state)    |
| Shooting             | Wind-Up Color (strobe)            |
| Ejecting             | Yellow (strobe)                   |
| Intaking             | Purple                            |
| Climbing             | Fire                              |
| Passive              | Dark Green (or white if low time) |

## CAN IDs
| Device                      | CAN ID |
|-----------------------------|--------|
| Front-Left Drive Motor      | 1      |
| Front-Left Steering Motor   | 2      |
| Front-Right Drive Motor     | 3      |
| Front-Right Steering Motor  | 4      |
| Back-Right Drive Motor      | 5      |
| Back-Right Steering Motor   | 6      |
| Back-Left Drive Motor       | 7      |
| Back-Left Steering Motor    | 8      |
| Climber Motor               | 9      |
| Intake/Launcher Left Motor  | 10     |
| Intake/Launcher Right Motor | 11     |
| Indexer Motor               | 12     |
| Pigeon 2                    | 13     |
| Power Distribution Hub      | 14     |
| CANdle (LEDs)               | 16     |

## Development
Here are some things to keep in mind while working on this codebase:

### Code Conventions/Best Practices
- In general, look at other code in this repository to see what to do
- At the very least:
    - Java naming conventions: https://www.geeksforgeeks.org/java/java-naming-conventions/
    - FRC-specific best practices: https://bovlb.github.io/frc-tips/commands/best-practices.html
- Some conventions are (at least partially) enforced:
    - Nullability: check out [JSpecify/NullAway](#jspecifynullaway)
    - Code style (indentation/spacing/etc): strictly enforced by [Spotless](#spotless)

### Working on Tasks
- Check out the [Project](https://github.com/orgs/PennridgeRobotics/projects/1)
- Select an issue in the "Ready" category
    - Assign the issue to yourself
    - Move the issue from "Ready" to "In progress"
- Create a new branch
    - Make sure the branch name contains that issue #
- When you're finished:
    - Move the issue from "In progress" to "In review"
    - Open a PR (with the issue #) and ask the programming lead to review & merge!

### Git & GitHub Usage
- Before working on code, **always pull** with `git pull` to ensure you have the latest changes
- Likewise, after working on code, **always push** with `git push` to upload your changes
- You should be committing your changes frequently and with descriptive commit names
- All new code must be developed on a new branch
  - When merging back into the `main` branch, make sure that your branch is up-to-date with `main`, then open a new PR!
  - All PRs must be approved by the programming lead in order to be merged
- Force pushes (`git push --force`) are **not** allowed (this will be enforced by GitHub)

### Spotless
- We use Spotless to maintain code formatting consistency
- Please ensure that you run Spotless before committing any code changes (otherwise the workflow will fail)
- You can do this by executing the `spotlessApply` Gradle task

### JSpecify/NullAway
- We use JSpecify annotations along with NullAway to enforce null-safety in our codebase
- Note that NullAway uses Error Prone to check nullability annotations at compile time

> [!IMPORTANT]
> - When adding new classes, make sure you annotate them with `@NullMarked`
> - This annotation means that all types within that class are non-nullable by default,
so you must use `@Nullable` explicitly for any nullable types
> - This annotation also tells NullAway to enforce null-safety checks within that class,
so it is important to add this annotation to every class that you create

Please check our document for more details: https://docs.google.com/document/d/1nnzGnt42i6uPzOHwAwAfORLo0D7Oy1VpVn8Up7hVth0/edit?tab=t.deskc5r0e8y9#heading=h.8nrizm8intq2

[BLine Path Inverter](https://pl.kotl.in/CLk4nuEjr)
