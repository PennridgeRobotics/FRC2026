package org.pennridge.robotics.frc.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

@NullMarked
public class SimSwerveSubsystem extends SubsystemBase implements SwerveSubsystem {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2D;

    public SimSwerveSubsystem() {
        final var simConfig = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofNav2X())
                .withSwerveModule(COTS.ofMark4(
                        DCMotor.getNEO(1),
                        DCMotor.getNEO(1),
                        Constants.DriveConstants.WHEEL_COEFFICIENT,
                        Constants.DriveConstants.SWERVE_GEAR_RATIO_LEVEL))
                .withTrackLengthTrackWidth(Constants.DriveConstants.TRACK_WIDTH, Constants.DriveConstants.TRACK_WIDTH)
                .withBumperSize(Constants.PhysicalConstants.ROBOT_LENGTH, Constants.PhysicalConstants.ROBOT_WIDTH);

        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(simConfig, new Pose2d(0, 0, new Rotation2d())));

        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        field2D = new Field2d();
        SmartDashboard.putData("Simulation Field", field2D);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                new Translation2d(),
                fieldRelative,
                true);
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), fieldRelative, true);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getActualPoseInSimulationWorld().getRotation();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        super.periodic();
        simulatedDrive.periodic();

        field2D.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2D.getObject("odometry").setPose(getPose());
    }
}
