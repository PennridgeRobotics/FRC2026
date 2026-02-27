package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.enums.Constants.FieldConstants;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

// Credits to
// https://github.com/FRCTeam360/RainMaker26/blob/main/src/main/java/frc/robot/subsystems/Shooter/ShotCalculator.java
@NullMarked
public class ShooterCalculator {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final InterpolatingDoubleTreeMap shooterDistanceVelocityMap = new InterpolatingDoubleTreeMap();
    private final DoublePublisher shotDistancePublisher;
    private final DoublePublisher shotVelocityPublisher;
    private final DoublePublisher shotHeadingPublisher;

    public ShooterCalculator(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;

        final var topicPrefix = "Shooter Calculator/";
        shotDistancePublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Shot Distance")
                .publish();
        shotVelocityPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Shot Velocity")
                .publish();
        shotHeadingPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Shot Heading")
                .publish();
    }

    public ShotData calculateVelocity() {
        final Pose2d robotPose = robotPoseSupplier.get();
        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d target = getTarget();
        final double distanceToTarget = target.getDistance(robotTranslation);

        final double targetVelocity = shooterDistanceVelocityMap.get(distanceToTarget);
        // rotate 180° because the shooter faces the back of the robot
        final Rotation2d targetHeading =
                target.minus(robotTranslation).getAngle().rotateBy(Rotation2d.k180deg);
        final var shot =
                new ShotData(Meters.of(distanceToTarget), RotationsPerSecond.of(targetVelocity), targetHeading);
        shotDistancePublisher.set(shot.distance().in(Meters));
        shotVelocityPublisher.set(shot.velocity().in(RotationsPerSecond));
        shotHeadingPublisher.set(shot.heading().getDegrees());
        return shot;
    }

    private void addDistanceVelocityData(Distance distance, AngularVelocity velocity) {
        shooterDistanceVelocityMap.put(distance.in(Meters), velocity.in(RotationsPerSecond));
    }

    private Translation2d getTarget() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return FieldConstants.HUB_RED;
        }
        return FieldConstants.HUB_BLUE;
    }

    public record ShotData(Distance distance, AngularVelocity velocity, Rotation2d heading) {}
}
