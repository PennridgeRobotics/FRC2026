package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.FuelSubsystem.OperatorFuelRequest;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutoManager;
import frc.robot.util.HubTracker;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.StringUtils;
import frc.robot.util.controller.CommandJoystickController;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkInput;
import frc.robot.util.dashboard.LoggedNetworkSendable;
import frc.robot.util.dashboard.LoggedNetworkStructArray;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.dashboard.Pigeon2Sendable;
import frc.robot.util.enums.Constants.ClimberConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.FuelConstants;
import frc.robot.util.enums.Constants.LightConstants;
import frc.robot.util.enums.Constants.MiscConstants;
import frc.robot.util.enums.SpeedMultiplier;
import frc.robot.vision.VisionManager;
import java.io.IOException;
import java.util.List;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class RobotContainer {
    // Initializes subsystems
    private final SwerveSubsystem swerveSubsystem;
    private final @Nullable LightsSubsystem lightsSubsystem;
    private final @Nullable FuelSubsystem fuelSubsystem;
    private final @Nullable ClimberSubsystem climberSubsystem;
    // private final @Nullable TankSubsystem tankSubsystem;

    private final ShooterCalculator shooterCalculator;
    private final PowerDistribution powerDistribution;
    private final VisionManager visionManager;
    private final MultiMotorInfoSendable motorInfo = new MultiMotorInfoSendable();
    private final @Nullable AutoManager autoManager;

    private final StructPublisher<Pose2d> aheadRobotPose;
    private final StructPublisher<Pose2d> behindRobotPose;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    private final @Nullable CommandJoystickController joystickController = ControllerConstants.USING_JOYSTICK
            ? new CommandJoystickController(ControllerConstants.JOYSTICK_CONTROLLER_PORT)
            : null;

    private final SendableChooser<AutoManager.AutoStartLocation> autoStartLocationChooser;
    private final LoggedNetworkBoolean autoShootAtStart = new LoggedNetworkBoolean("/Auto/1. Shoot at Start", true);
    private final LoggedNetworkBoolean autoCollectFromMid = new LoggedNetworkBoolean("/Auto/2. Collect From Mid", true);
    private final LoggedNetworkBoolean autoDepot = new LoggedNetworkBoolean("/Auto/3. Auto Depot", false);
    private final LoggedNetworkBoolean autoOutpost = new LoggedNetworkBoolean("/Auto/4. Auto Outpost", false);
    private final LoggedNetworkBoolean autoClimb = new LoggedNetworkBoolean("/Auto/5. Auto Climb", false);

    private final LoggedNetworkBoolean useOdometry = new LoggedNetworkBoolean("/Misc/Use Odometry", true);
    private final Trigger useOdometryTrigger = new Trigger(useOdometry);
    private final Field2d field2d = new Field2d();
    private final LoggedNetworkSendable<Field2d> loggedField = new LoggedNetworkSendable<>("/Misc/Field", field2d);

    private final LoggedNetworkStructArray<Pose2d> loggedBLineTrajectory =
            new LoggedNetworkStructArray<>("/Misc/BLine Trajectory", Pose2d.struct, new Pose2d[0]);

    /** The container for the robot. Contains subsystems, I/O devices, and commands. */
    public RobotContainer() {
        powerDistribution =
                new PowerDistribution(MiscConstants.POWER_DISTRIBUTION_HUB_ID, PowerDistribution.ModuleType.kRev);
        try {
            swerveSubsystem = new SwerveSubsystem(motorInfo);
        } catch (IOException ex) {
            final var finalException =
                    new RuntimeException("Error instantiating Swerve Subsystem: " + ex.getMessage(), ex);
            DriverStation.reportError(
                    "Error instantiating Swerve Subsystem: " + ex.getMessage(), finalException.getStackTrace());
            throw finalException;
        }
        visionManager = swerveSubsystem.getVisionManager();
        shooterCalculator = swerveSubsystem.getShooterCalculator();
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator, motorInfo) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem(motorInfo) : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

        if (fuelSubsystem != null) {
            swerveSubsystem.setIsShootingTrigger(
                    fuelSubsystem.isWindingUpTrigger().or(fuelSubsystem.isLaunchingTrigger()));
        }

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoStartLocationChooser = new SendableChooser<>();
        for (final var location : AutoManager.AutoStartLocation.values()) {
            autoStartLocationChooser.addOption(StringUtils.capitalizeFully(location.name()), location);
        }

        autoManager = (fuelSubsystem != null)
                ? new AutoManager(swerveSubsystem, swerveSubsystem.getPathBuilder(), fuelSubsystem, climberSubsystem)
                : null;

        aheadRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Ahead", Pose2d.struct)
                .publish();
        behindRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Behind", Pose2d.struct)
                .publish();

        configureBindings();

        initSmartDashboard();

        HubTracker.isActive(); // initialize HubTracker
    }

    public @Nullable Command getAutonomousCommand() {
        return autoManager != null
                ? autoManager.getAutoCommand(new AutoManager.AutoOptions(
                        autoStartLocationChooser.getSelected(),
                        autoShootAtStart.getAsBoolean(),
                        autoDepot.getAsBoolean(),
                        autoOutpost.getAsBoolean(),
                        autoClimb.getAsBoolean(),
                        autoCollectFromMid.getAsBoolean()))
                : null;
    }

    public void periodic() {
        updateField();
        updateAheadRobotPose();
    }

    private void updateAheadRobotPose() {
        aheadRobotPose.set(swerveSubsystem.getRobotPose().plus(new Transform2d(100.0, 0, Rotation2d.kZero)));
        behindRobotPose.set(swerveSubsystem.getRobotPose().plus(new Transform2d(-100.0, 0, Rotation2d.kZero)));
    }

    private void configureBindings() {
        /*swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                driverController::getLeftY, () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

        driverController.start().onTrue(swerveSubsystem.resetYaw());*/

        if (joystickController != null) {
            swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                    () -> -joystickController.getY(),
                    () -> -joystickController.getX(),
                    () -> -joystickController.getTwist()));
            if (fuelSubsystem != null) {
                joystickController
                        .trigger(false, false)
                        .whileTrue(fuelSubsystem.launchCommand(true))
                        .whileTrue(swerveSubsystem.faceTowardsHubCommand());
                joystickController
                        .trigger(false, true)
                        .whileTrue(fuelSubsystem.windUpCommand())
                        .whileTrue(swerveSubsystem.faceTowardsHubCommand());
                joystickController.topHat(false, false).whileTrue(fuelSubsystem.intakeCommand());
            }
            if (autoManager != null) {
                joystickController.a1().whileTrue(autoManager.testAuto());
                joystickController.a2().whileTrue(autoManager.testOnePointPath());

                joystickController.b1().whileTrue(swerveSubsystem.lockPoseCommand());
            }
            return;
        }

        // for testing
        final var fieldOriented = true;
        final var forceRobotOrientedRotation = true;
        if (fieldOriented) {
            if (forceRobotOrientedRotation) {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldAndRobotOrientedCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> /*-operatorController.getLeftX()*/ 0,
                        () -> /*-operatorController.getLeftY()*/ 0));
                driverController.rightStick().whileTrue(swerveSubsystem.lockYawTowardsVelocity());
            } else {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedHeadingCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> -driverController.getRightY()));
            }
        } else {
            swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOrientedCommand(
                    () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                    () -> MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                    () -> -driverController.getRightX()));
        }

        driverController.leftBumper().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.SLOW));
        driverController
                .leftBumper()
                .negate()
                .and(driverController.rightTrigger().negate())
                .onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.NORMAL));
        driverController.rightTrigger().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.FAST));
        driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
        driverController.y().whileTrue(swerveSubsystem.faceTowardsHubCommand());
        driverController.x().whileTrue(swerveSubsystem.lockPoseCommand());
        if (fuelSubsystem != null) {
            driverController
                    .b()
                    .and(operatorController.leftTrigger().negate()) // let operator override
                    .whileTrue(fuelSubsystem.launchCommand(true))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            driverController
                    .leftTrigger()
                    .and(operatorController.leftTrigger().negate()) // let operator override
                    .whileTrue(fuelSubsystem.intakeCommand());
        }
        /*driverController
        .a()
        .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                PositionCalibrationLocation.FRONT_LEFT_OF_HUB));*/

        operatorController
                .rightBumper()
                .and(operatorController.start())
                .whileTrue(swerveSubsystem.straightenWheelsCommand());
        operatorController
                .leftTrigger()
                .and(operatorController.start())
                .onTrue(swerveSubsystem.toggleUseBackCameraInPoseEstimation());
        operatorController
                .rightTrigger()
                .and(operatorController.start())
                .onTrue(swerveSubsystem.toggleUseFrontCameraInPoseEstimation());

        operatorController.rightStick().onTrue(swerveSubsystem.toggleForceNormalDriveMode());

        if (autoManager != null) {
            operatorController.leftBumper().and(operatorController.start()).whileTrue(autoManager.testOnePointPath());
        }

        if (fuelSubsystem != null) {
            operatorController
                    .rightBumper()
                    .and(operatorController.start().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.WINDUP));
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_WINDUP))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_NO_WINDUP))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            operatorController.leftBumper().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.EJECT));
            operatorController.a().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.INTAKE));
            operatorController.b().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.UNJAM));
            operatorController.leftStick().toggleOnTrue(shooterCalculator.temporarilyEnableManualMode());
            new Trigger(() -> operatorController.getLeftX() < -0.5)
                    .whileTrue(shooterCalculator.decreaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getLeftX() > 0.5)
                    .whileTrue(shooterCalculator.increaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getRightX() < -0.5)
                    .whileTrue(shooterCalculator.decreaseVelocityOffset());
            new Trigger(() -> operatorController.getRightX() > 0.5)
                    .whileTrue(shooterCalculator.increaseVelocityOffset());
            operatorController.back().whileTrue(fuelSubsystem.temporarilyUseMaxPower());
        }
        if (climberSubsystem != null) {
            operatorController
                    .y()
                    .whileTrue(climberSubsystem.climbCommand(
                            operatorController.start().negate(), operatorController.back()));
            operatorController
                    .x()
                    .whileTrue(climberSubsystem.armCommand(
                            operatorController.start().negate(), operatorController.back()));
        }
        /*if (autoManager != null) {
            operatorController.back().whileTrue(autoManager.testAuto());
        }*/
    }

    public void initSmartDashboard() {
        new LoggedNetworkSendable<>("/Auto/Start Location Chooser", autoStartLocationChooser);
        new LoggedNetworkSendable<>("/Misc/Power Distribution", powerDistribution);
        new LoggedNetworkSendable<>("/Misc/Motor Info", motorInfo);
        new LoggedNetworkSendable<>("/Pigeon2", new Pigeon2Sendable(swerveSubsystem.getPigeon2()));
        /*final var emptyPoseArray = new Pose2d[0];
        new LoggedNetworkStructArray<>("/Misc/BLine Completed Poses", Pose2d.struct, () -> {
            if (autoManager == null) return emptyPoseArray;
            final var completedPoses = autoManager.getCompletedPoses();
            if (completedPoses == null) return emptyPoseArray;
            return completedPoses.toArray(new Pose2d[0]);
        });
        new LoggedNetworkStructArray<>("/Misc/BLine Poses to Complete", Pose2d.struct, () -> {
            if (autoManager == null) return emptyPoseArray;
            final var posesToComplete = autoManager.getPosesToComplete();
            if (posesToComplete == null) return emptyPoseArray;
            return posesToComplete.toArray(new Pose2d[0]);
        });*/
    }

    private void updateField() {
        if (autoManager == null) return;
        field2d.setRobotPose(swerveSubsystem.getRobotPose());
        final FieldObject2d trajectoryObject = field2d.getObject("BLine trajectory");
        final List<Pose2d> currentTrajectory = autoManager.getCurrentPoses();
        if (currentTrajectory == null) {
            trajectoryObject.setPoses();
            loggedBLineTrajectory.set(new Pose2d[0]);
            return;
        }
        trajectoryObject.setPoses(currentTrajectory);
        loggedBLineTrajectory.set(currentTrajectory.toArray(Pose2d[]::new));
    }

    public void preSchedulerUpdate() {
        shooterCalculator.prePeriodic();
        LoggedNetworkInput.runAllPeriodic();
    }

    public void postSchedulerUpdate() {
        // NetworkTableInstance.getDefault().flush();
    }

    public void autonomousInit() {
        if (autoManager != null) {
            autoManager.autonomousInit();
        }
    }

    public void teleopInit() {
        if (autoManager != null) {
            autoManager.teleopInit();
        }
    }

    public void simulationInit() {
        shooterCalculator.simulationInit();
    }

    public void simulationPeriodic() {
        shooterCalculator.simulationPeriodic();
    }
}
