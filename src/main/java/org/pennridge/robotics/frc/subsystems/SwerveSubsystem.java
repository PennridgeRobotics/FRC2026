package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
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
        swerveDrive.setHeadingCorrection(false); // only while controlling the robot via angle
        swerveDrive.setCosineCompensator(SwerveDriveTelemetry.isSimulation); // disable for simulations
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // may need to adjust; see docs
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // can set to true, but I want to test
        swerveDrive.setMotorIdleMode(false);

        initSmartDashboard();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        updatePhotonVision();
    }

    /**
     * @param xVelocity Positive = towards other alliance
     * @param yVelocity Positive = towards left wall
     * @param angularVelocity Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final Supplier<AngularVelocity> angularVelocity) {
        return run(() -> driveFieldOriented(xVelocity.get(), yVelocity.get(), angularVelocity.get()));
    }

    /**
     * @param xInput Positive = towards other alliance
     * @param yInput Positive = towards left wall
     * @param angularInput Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final DoubleSupplier xInput, final DoubleSupplier yInput, final DoubleSupplier angularInput) {
        return driveFieldOrientedCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                () -> joystickToAngularVelocity(angularInput.getAsDouble()));
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
