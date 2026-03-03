package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.enums.Constants.FieldConstants;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

// Credits to
// https://github.com/FRCTeam360/RainMaker26/blob/main/src/main/java/frc/robot/subsystems/Shooter/ShotCalculator.java
@NullMarked
public class ShooterCalculator {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final InterpolatingDoubleTreeMap shooterDistanceVelocityMap = new InterpolatingDoubleTreeMap();
    private final Map<Double, Double> savedShooterDistanceVelocityMap =
            new TreeMap<>(); // because InterpolatingDoubleTreeMap won't let us extract its values
    private @Nullable ShotData lastShotData;
    private boolean lastShotDataValidForCache;

    private final DoublePublisher shotDistancePublisher;
    private final DoublePublisher shotVelocityPublisher;
    private final DoublePublisher shotHeadingPublisher;
    private final BooleanPublisher cachedPublisher;
    private final StringEntry savedShooterDistanceVelocityMapEntry;
    private final BooleanEntry saveCurrentDataButtonEntry;

    private static final String NO_DATA_TEXT = "(No Data)";

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
        cachedPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Used Cached Shot Data")
                .publish();
        savedShooterDistanceVelocityMapEntry = NetworkTableInstance.getDefault()
                .getStringTopic(topicPrefix + "Saved Shooter Distance Velocity Map")
                .getEntry(NO_DATA_TEXT);
        savedShooterDistanceVelocityMapEntry.set(NO_DATA_TEXT);
        saveCurrentDataButtonEntry = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Save Current Distance and Velocity")
                .getEntry(false);
        saveCurrentDataButtonEntry.set(false);

        addPreviouslySavedData();
    }

    private void addPreviouslySavedData() {
        // Generated from https://docs.google.com/spreadsheets/d/1CAVIv4i_sHdjmRlEvcaaNroXZzFWiLivQj3PQpYls7k

    }

    public ShotData calculateVelocity() {
        if (lastShotDataValidForCache && lastShotData != null) {
            cachedPublisher.set(true);
            return lastShotData;
        }
        cachedPublisher.set(false);
        final Pose2d robotPose = robotPoseSupplier.get();
        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d target = getTarget();
        final double distanceToTarget = target.getDistance(robotTranslation);

        final double targetVelocity = Objects.requireNonNullElse(shooterDistanceVelocityMap.get(distanceToTarget), 0.0);
        // rotate 180° because the shooter faces the back of the robot
        final Rotation2d targetHeading =
                target.minus(robotTranslation).getAngle().rotateBy(Rotation2d.k180deg);
        final var shot =
                new ShotData(Meters.of(distanceToTarget), RotationsPerSecond.of(targetVelocity), targetHeading);
        shotDistancePublisher.set(shot.distance().in(Meters));
        shotVelocityPublisher.set(shot.velocity().in(RotationsPerSecond));
        shotHeadingPublisher.set(shot.heading().getDegrees());
        lastShotData = shot;
        lastShotDataValidForCache = true;
        return shot;
    }

    public void addCurrentDataToMap(AngularVelocity shooterVelocity) {
        final Pose2d robotPose = robotPoseSupplier.get();
        final double distanceToTarget = getTarget().getDistance(robotPose.getTranslation());
        addRawDistanceVelocityData(distanceToTarget, shooterVelocity.in(RotationsPerSecond));
    }

    private void addDistanceVelocityData(Distance distance, AngularVelocity velocity) {
        addRawDistanceVelocityData(distance.in(Meters), velocity.in(RotationsPerSecond));
    }

    private void addRawDistanceVelocityData(double distance, double velocity) {
        System.out.println("Added distance " + distance + "m with velocity " + velocity + "rot/s");
        shooterDistanceVelocityMap.put(distance, velocity);
        savedShooterDistanceVelocityMap.put(distance, velocity);
        savedShooterDistanceVelocityMapEntry.set(exportData());
    }

    private Translation2d getTarget() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return FieldConstants.HUB_RED;
        }
        return FieldConstants.HUB_BLUE;
    }

    private String exportData() {
        final var entries = savedShooterDistanceVelocityMap.entrySet();
        if (entries.isEmpty()) return NO_DATA_TEXT;
        return entries.stream().map(e -> e.getKey() + "," + e.getValue()).reduce("", (a, b) -> a + ";" + b);
    }

    private void importData() {
        shooterDistanceVelocityMap.clear(); // clear the interpolation map
        savedShooterDistanceVelocityMap.clear(); // clear the saved map
        final var data = savedShooterDistanceVelocityMapEntry.get();
        if (data.equals(NO_DATA_TEXT)) {
            return;
        }
        final var entries = data.split(";");
        for (var entry : entries) {
            final var split = entry.split(",");
            if (split.length != 2) continue;
            final var distance = Double.parseDouble(split[0]);
            final var velocity = Double.parseDouble(split[1]);
            addRawDistanceVelocityData(distance, velocity);
        }
    }

    public void prePeriodic() {
        // lastShotData = null;
        lastShotDataValidForCache = false;

        if (saveCurrentDataButtonEntry.get()) {
            // button pressed
            saveCurrentDataButtonEntry.set(false);
            if (lastShotData == null) return;
            addDistanceVelocityData(lastShotData.distance(), lastShotData.velocity());
        }

        final var currentDashboardData = savedShooterDistanceVelocityMapEntry.get();
        final var exportedData = exportData();
        if (!currentDashboardData.equals(exportedData)) {
            importData();
        }
    }

    public record ShotData(Distance distance, AngularVelocity velocity, Rotation2d heading) {}
}
