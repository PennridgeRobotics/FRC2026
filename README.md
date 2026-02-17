# FRC2026
Welcome to Ridge Robotics' repository for the FRC 2026 season!

## Controls
| Button | Action |
|--------|--------|
| TBD    | TBD    |

## CAN IDs
| Device                       | CAN ID |
|------------------------------|--------|
| Front-Left Drive Motor       | 1      |
| Front-Left Steering Motor    | 2      |
| Front-Left Absolute Encoder  | 9      |
| Front-Right Drive Motor      | 3      |
| Front-Right Steering Motor   | 4      |
| Front-Right Absolute Encoder | 10     |
| Back-Right Drive Motor       | 5      |
| Back-Right Steering Motor    | 6      |
| Back-Right Absolute Encoder  | 11     |
| Back-Left Drive Motor        | 7      |
| Back-Left Steering Motor     | 8      |
| Back-Left Absolute Encoder   | 12     |
| Pigeon 2                     | 13     |
| Intake/Launcher Motor        | 14     |
| Feeder Motor                 | 15     |
| CANdle (LEDs)                | 16     |

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
    - Common possible points of error: checked by [Error Prone](#error-prone)

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

### Error Prone
- We use Error Prone (by Google) to catch common Java mistakes at compile time
- Unlike Spotless, Error Prone is executed whenever the code is compiled
- Note that some checks will only show as a warning, but others will fail the compilation

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
