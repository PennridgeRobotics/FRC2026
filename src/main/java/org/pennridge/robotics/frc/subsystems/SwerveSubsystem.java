package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.Robot;
import org.pennridge.robotics.frc.util.dashboard.PIDSendable;
import org.pennridge.robotics.frc.util.enums.Constants.BLineConstants;
import org.pennridge.robotics.frc.util.enums.Constants.ControllerConstants;
import org.pennridge.robotics.frc.util.enums.Constants.DriveConstants;
import org.pennridge.robotics.frc.util.enums.Constants.FieldConstants;
import org.pennridge.robotics.frc.util.enums.Constants.VisionConstants;
import org.pennridge.robotics.frc.util.enums.DriveMode;
import org.pennridge.robotics.frc.vision.PhotonCamera;
import org.pennridge.robotics.frc.vision.VisionManager;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

@NullMarked
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private VisionManager visionManager;

    private DriveMode currentDriveMode = DriveMode.NORMAL; // automatically accounts for forceNormalDriveMode
    private boolean forceNormalDriveMode = false;

    private final Trigger inBumpZoneTrigger;
    private final Trigger onBump;

    private final PIDController bLineTranslationPID = new PIDController(5.0, 0, 0);
    private final PIDController bLineRotationPID = new PIDController(3.0, 0, 0);
    private final PIDController bLineCrossTrackPID = new PIDController(2.0, 0, 0);
    private final FollowPath.Builder pathBuilder;

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

        inBumpZoneTrigger =
                new Trigger(this::isInBumpZone).and(() -> !forceNormalDriveMode).debounce(0.1);
        inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK, () -> "entered bump zone"));
        inBumpZoneTrigger.onFalse(updateDriveMode(DriveMode.NORMAL, () -> "left bump zone"));

        onBump = inBumpZoneTrigger
                .and(() -> {
                    final var rotation3d = swerveDrive.getGyroRotation3d();
                    final var angle =
                            Math.toDegrees(new Rotation3d(rotation3d.getX(), rotation3d.getY(), 0).getAngle());
                    final var rollVel = Math.abs(getPigeon2()
                            .getAngularVelocityXWorld(false)
                            .getValue()
                            .in(DegreesPerSecond));
                    final var pitchVel = Math.abs(getPigeon2()
                            .getAngularVelocityYWorld(false)
                            .getValue()
                            .in(DegreesPerSecond));
                    final var angularVelocity = Math.hypot(rollVel, pitchVel);
                    return angle > 7.0 || (angle > 2.0 && angularVelocity > 50);
                })
                .debounce(0.25, DebounceType.kBoth);
        onBump.onTrue(runOnce(() -> System.out.println("on bump")));
        onBump.onFalse(
                updateDriveMode(DriveMode.NORMAL, () -> inBumpZoneTrigger.getAsBoolean() ? "no longer on bump" : null));

        setupVisionManager();
        pathBuilder = setupBLine();
        initSmartDashboard();
    }

    private void updateOdometry() {
        swerveDrive.updateOdometry();
        visionManager.updatePoseEstimation(swerveDrive);
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

    public Command updateDriveMode(DriveMode newMode, @Nullable Supplier<@Nullable String> cause) {
        return runOnce(() -> {
            if (cause != null && cause.get() != null) {
                System.out.println("Updated drive mode to " + newMode + " (" + cause.get() + ")");
            }
            currentDriveMode = newMode;
        });
    }

    public Command updateDriveMode(DriveMode newMode) {
        return updateDriveMode(newMode, null);
    }

    public Command forceNormalDriveMode(boolean force) {
        return runOnce(() -> forceNormalDriveMode = force);
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
        final AngularVelocity finalAngularVelocity =
                switch (currentDriveMode) {
                    case NORMAL -> angularVelocity;
                    case BUMP_LOCK -> getTargetAngularVelocity(getBumpLockAngle());
                };
        swerveDrive.driveFieldOriented(new ChassisSpeeds(adjustedXVelocity, adjustedYVelocity, finalAngularVelocity));
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

    private AngularVelocity getTargetAngularVelocity(Rotation2d targetAngle) {
        final var currentHeading = swerveDrive.getOdometryHeading().getRadians();
        final var targetHeading = targetAngle.getRadians();
        final var maxAngularVelocity = getMaximumChassisAngularVelocity().in(RadiansPerSecond);
        final var calculated = swerveDrive.swerveController.thetaController.calculate(currentHeading, targetHeading)
                * maxAngularVelocity;
        final var limited = swerveDrive.swerveController.angleLimiter != null
                ? swerveDrive.swerveController.angleLimiter.calculate(calculated)
                : calculated;
        return RadiansPerSecond.of(limited);
    }

    private Rotation2d calculateTargetAngle(double headingX, double headingY) {
        return Rotation2d.fromRadians(
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

    private void setModuleOrientations(Rotation2d rotation) {
        final var states = new SwerveModuleState[swerveDrive.getModules().length];
        Arrays.fill(states, new SwerveModuleState(0, rotation));
        swerveDrive.setModuleStates(states, false);
    }

    private FollowPath.Builder setupBLine() {
        Path.setDefaultGlobalConstraints(BLineConstants.GLOBAL_CONSTRAINTS);
        SmartDashboard.putData(
                "BLine Translation PID",
                new PIDSendable(
                        bLineTranslationPID, PIDSendable.Type.PID, PIDSendable.PIDValues.from(bLineTranslationPID)));
        SmartDashboard.putData(
                "BLine Rotation PID",
                new PIDSendable(bLineRotationPID, PIDSendable.Type.PID, PIDSendable.PIDValues.from(bLineRotationPID)));
        SmartDashboard.putData(
                "BLine Cross Track PID",
                new PIDSendable(
                        bLineCrossTrackPID, PIDSendable.Type.PID, PIDSendable.PIDValues.from(bLineCrossTrackPID)));
        return new FollowPath.Builder(
                        this,
                        this::getRobotPose,
                        swerveDrive::getRobotVelocity,
                        swerveDrive::drive,
                        bLineTranslationPID,
                        bLineRotationPID,
                        bLineCrossTrackPID)
                .withDefaultShouldFlip()
                .withPoseReset(this::resetPose);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData("Swerve Subsystem", builder -> {
            builder.addStringProperty("Drive Mode", currentDriveMode::getFriendlyName, null);
            builder.addStringProperty(
                    "Bump Status",
                    () -> {
                        if (currentDriveMode == DriveMode.NORMAL) return Color.kLime.toHexString();
                        if (onBump.getAsBoolean()) return Color.kRed.toHexString();
                        return Color.kYellow.toHexString(); // in bump area, but not on the bump itself
                    },
                    null);
        });
    }

    private void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

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

    private void setupVisionManager() {
        this.visionManager = new VisionManager(() -> swerveDrive
                .getSimulationDriveTrainPose()
                .orElseThrow(
                        () -> new IllegalStateException("Cannot get simulation drive train pose when not simulating")));

        swerveDrive.stopOdometryThread();
        @SuppressWarnings("resource")
        final var odometryThread = new Notifier(this::updateOdometry);
        odometryThread.setName("Odometry Thread");
        odometryThread.startPeriodic(Robot.isSimulation() ? 0.01 : 0.02);

        if (!VisionConstants.VISION_ENABLED) {
            return;
        }
        visionManager.addCamera(new PhotonCamera(
                VisionConstants.CAMERA_1_NAME,
                VisionConstants.CAMERA_1_TRANSLATION,
                VisionConstants.CAMERA_1_ROTATION));
    }

    // check every 45° angle to find the quickest one that we can rotate to
    private Rotation2d getBumpLockAngle() {
        for (int angle = -135; angle < 180; angle += 90) {
            if (Math.abs(MathUtil.inputModulus(getRobotPose().getRotation().getDegrees() - angle, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(angle);
            }
        }
        return Rotation2d.kZero;
    }

    private boolean isInBumpZone() {
        final var pose = getRobotPose().getTranslation();
        for (var zone : FieldConstants.BUMP_ZONES) {
            if (zone.contains(pose)) {
                return true;
            }
        }
        return false;
    }

    private CorePigeon2 getPigeon2() {
        return (CorePigeon2) swerveDrive.getGyro().getIMU();
    }
}
