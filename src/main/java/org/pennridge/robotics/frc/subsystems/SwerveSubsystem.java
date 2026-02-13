package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.manager.VisionManager;
import org.pennridge.robotics.frc.util.enums.Constants.ControllerConstants;
import org.pennridge.robotics.frc.util.enums.Constants.DriveConstants;
import org.pennridge.robotics.frc.util.enums.Constants.VisionConstants;
import org.pennridge.robotics.frc.util.lib.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

@NullMarked
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private @Nullable VisionManager visionManager;

    @SuppressWarnings("StaticAssignmentInConstructor")
    public SwerveSubsystem() throws IOException {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(
                        new File(Filesystem.getDeployDirectory(), DriveConstants.SWERVE_CONFIG_DIRECTORY))
                .createSwerveDrive(
                        DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.kZero));
        swerveDrive.setHeadingCorrection(false); // enable this after testing/tuning PID
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // disable for simulations
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // may need to adjust; see docs
        swerveDrive.useExternalFeedbackSensor();
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // can set to true, but I want to test
        swerveDrive.setMotorIdleMode(true);

        // setupPathPlanner();
        initSmartDashboard();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        updatePhotonVision();
    }

    /**
     * @param xVelocity Positive = towards the other alliance
     * @param yVelocity Positive = towards the left wall
     * @param angularVelocity Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final Supplier<AngularVelocity> angularVelocity) {
        return run(() -> driveFieldOriented(xVelocity.get(), yVelocity.get(), angularVelocity.get()));
    }

    /**
     * @param xInput [-1,1] Positive = towards the other alliance
     * @param yInput [-1,1] Positive = towards the left wall
     * @param angularInput [-1,1] Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final DoubleSupplier xInput, final DoubleSupplier yInput, final DoubleSupplier angularInput) {
        return driveFieldOrientedCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                () -> joystickToAngularVelocity(angularInput.getAsDouble()));
    }

    /**
     * @param xVelocity Positive = towards the other alliance
     * @param yVelocity Positive = towards the left wall
     * @param headingX [-1,1] Heading X (positive = front)
     * @param headingY [-1,1] Heading Y (positive = left)
     */
    public Command driveFieldOrientedCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final DoubleSupplier headingX,
            final DoubleSupplier headingY) {
        return run(() ->
                driveFieldOriented(xVelocity.get(), yVelocity.get(), headingX.getAsDouble(), headingY.getAsDouble()));
    }

    /**
     * @param xInput [-1,1] Positive = towards the other alliance
     * @param yInput [-1,1] Positive = towards the left wall
     * @param headingX [-1,1] Heading X (positive = front)
     * @param headingY [-1,1] Heading Y (positive = left)
     */
    public Command driveFieldOrientedCommand(
            final DoubleSupplier xInput,
            final DoubleSupplier yInput,
            final DoubleSupplier headingX,
            final DoubleSupplier headingY) {
        return driveFieldOrientedCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                headingX,
                headingY);
    }

    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(mod -> mod.setAngle(0.0)));
    }

    public Command lockPoseCommand() {
        return run(swerveDrive::lockPose);
    }

    public Command sysIdDriveMotorCommand() {
        // Empty config defaults to 1 Volt/sec ramp rate and 7 Volt step voltage
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12, true),
                3.0,
                5.0,
                3.0);
    }

    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive), 3.0, 5.0, 3.0);
    }

    public void zeroGyroWithAlliance() {
        swerveDrive.zeroGyro();
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().orElse(null))) {
            swerveDrive.resetOdometry(new Pose2d(getRobotPose().getTranslation(), Rotation2d.k180deg));
        }
    }

    private void driveFieldOriented(
            final LinearVelocity xVelocity,
            final LinearVelocity yVelocity,
            final double headingX,
            final double headingY) {
        driveFieldOriented(xVelocity, yVelocity, getTargetAngularVelocity(calculateTargetAngle(headingX, headingY)));
    }

    private void driveFieldOriented(
            final LinearVelocity xVelocity, final LinearVelocity yVelocity, final AngularVelocity angularVelocity) {
        final var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }
        final var shouldFlip = alliance.get() == DriverStation.Alliance.Red;
        final var adjustedXVelocity = shouldFlip ? xVelocity.unaryMinus() : xVelocity;
        final var adjustedYVelocity = shouldFlip ? yVelocity.unaryMinus() : yVelocity;
        swerveDrive.driveFieldOriented(new ChassisSpeeds(adjustedXVelocity, adjustedYVelocity, angularVelocity));
    }

    public Command driveFieldOrientedTestCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of
        // control.
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    private AngularVelocity getTargetAngularVelocity(Angle targetAngle) {
        final var currentHeading = swerveDrive.getOdometryHeading().getRadians();
        final var targetHeading = targetAngle.in(Radians);
        final var maxAngularVelocity = getMaximumChassisAngularVelocity().in(RadiansPerSecond);
        final var calculated = swerveDrive.swerveController.thetaController.calculate(currentHeading, targetHeading)
                * maxAngularVelocity;
        final var limited = swerveDrive.swerveController.angleLimiter != null
                ? swerveDrive.swerveController.angleLimiter.calculate(calculated)
                : calculated;
        return RadiansPerSecond.of(limited);
    }

    private Angle calculateTargetAngle(double headingX, double headingY) {
        return Radians.of(
                swerveDrive.swerveController.withinHypotDeadband(headingX, headingY)
                        ? swerveDrive.swerveController.lastAngleScalar
                        : Math.atan2(headingX, headingY));
    }

    private LinearVelocity joystickToLinearVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.pow(withDeadband, 3);
        return getMaximumChassisVelocity().times(scaled);
    }

    private AngularVelocity joystickToAngularVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.pow(withDeadband, 3);
        return getMaximumChassisAngularVelocity().times(scaled);
    }

    private void updatePhotonVision() {
        if (visionManager == null) {
            return;
        }
        visionManager.updatePoseEstimation(swerveDrive);
    }

    private void updateLimelightVision() {
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        final var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
        if (Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 360) {
            return;
        }
        if (estimate == null || estimate.tagCount == 0) {
            return;
        }
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        swerveDrive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    private void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    // Robot pose supplier
                    swerveDrive::getPose,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    swerveDrive::resetOdometry,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    swerveDrive::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs
                    // individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built-in path following controller for holonomic drive
                            // trains
                            // Translation PID
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Rotation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)),
                    // The robot configuration
                    config,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        return alliance.filter(value -> value == DriverStation.Alliance.Red)
                                .isPresent();
                    },
                    this
                    // Reference to this subsystem to set requirements
                    );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    private void initSmartDashboard() {}

    private SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    private Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    private LinearVelocity getMaximumChassisVelocity() {
        return MetersPerSecond.of(swerveDrive.getMaximumChassisVelocity());
    }

    private AngularVelocity getMaximumChassisAngularVelocity() {
        return RadiansPerSecond.of(swerveDrive.getMaximumChassisAngularVelocity());
    }

    public VisionManager setupVisionManager() {
        final var visionManager = new VisionManager(swerveDrive::getPose, swerveDrive.field);
        this.visionManager = visionManager;
        return visionManager;
    }
}
