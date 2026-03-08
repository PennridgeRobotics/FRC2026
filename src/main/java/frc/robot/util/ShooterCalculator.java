package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.FieldConstants;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import redempt.crunch.CompiledExpression;
import redempt.crunch.Crunch;
import redempt.crunch.exceptions.ExpressionCompilationException;
import redempt.crunch.exceptions.ExpressionEvaluationException;
import redempt.crunch.functional.EvaluationEnvironment;

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
    private CalculationMode calculationMode = CalculationMode.EQUATION;
    private String originalExpression = "-45.66 * x^(-2.622) + 56.778";
    private CompiledExpression compiledExpression = Crunch.compileExpression("0");
    private long lastUpdateTimestampMillis;

    private final DoublePublisher shotDistancePublisher;
    private final DoublePublisher shotVelocityPublisher;
    private final DoublePublisher shotHeadingPublisher;
    private final DoublePublisher invertedShotHeadingPublisher;
    private final BooleanPublisher cachedPublisher;
    private final DoublePublisher savedDataCountPublisher;
    private final StringEntry savedShooterDistanceVelocityMapEntry;
    private final BooleanEntry saveCurrentDataButtonEntry;
    private final BooleanEntry exportToConsoleButton;
    private final StringSubscriber equationSubscriber;
    private final DoubleEntry velocityOffsetEntry;

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
        invertedShotHeadingPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Inverted Shot Heading")
                .publish();
        cachedPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Used Cached Shot Data")
                .publish();
        savedDataCountPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Saved Data Count")
                .publish();
        savedShooterDistanceVelocityMapEntry = NetworkTableInstance.getDefault()
                .getStringTopic(topicPrefix + "Saved Shooter Distance Velocity Map")
                .getEntry(NO_DATA_TEXT);
        savedShooterDistanceVelocityMapEntry.set(NO_DATA_TEXT);
        saveCurrentDataButtonEntry = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Save Current Distance and Velocity")
                .getEntry(false);
        saveCurrentDataButtonEntry.set(false);
        exportToConsoleButton = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Export to Console")
                .getEntry(false);
        exportToConsoleButton.set(false);
        var equationEntry = NetworkTableInstance.getDefault()
                .getStringTopic(topicPrefix + "Equation")
                .getEntry(originalExpression);
        equationEntry.set(originalExpression);
        equationSubscriber = equationEntry.getTopic().subscribe(originalExpression);
        compileEquation();
        velocityOffsetEntry = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Velocity Offset")
                .getEntry(0.0);
        velocityOffsetEntry.set(0.0);

        SmartDashboard.putData(
                topicPrefix + "/SmartDashboard/Calculation Mode",
                SplitButtonChooser.withEnum(
                        () -> calculationMode,
                        Set.of(newMode -> calculationMode = newMode),
                        calculationMode,
                        CalculationMode.class));

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
        lastUpdateTimestampMillis = System.currentTimeMillis();
        final Pose2d robotPose = robotPoseSupplier.get();
        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d target = getTarget();
        final double distanceToTarget = target.getDistance(robotTranslation);

        final double targetVelocity = calculateAngularVelocity(distanceToTarget) + velocityOffsetEntry.get();
        // rotate 180° because the shooter faces the back of the robot
        final Rotation2d targetHeading =
                target.minus(robotTranslation).getAngle().rotateBy(Rotation2d.k180deg);
        final var shot =
                new ShotData(Meters.of(distanceToTarget), RotationsPerSecond.of(targetVelocity), targetHeading);
        shotDistancePublisher.set(shot.distance().in(Meters));
        shotVelocityPublisher.set(shot.velocity().in(RotationsPerSecond));
        shotHeadingPublisher.set(shot.heading().getDegrees());
        invertedShotHeadingPublisher.set(-(shot.heading.getDegrees() + 180));
        lastShotData = shot;
        lastShotDataValidForCache = true;
        return shot;
    }

    public void addCurrentDataToMap(AngularVelocity shooterVelocity) {
        final Pose2d robotPose = robotPoseSupplier.get();
        final double distanceToTarget = Math.round(getTarget().getDistance(robotPose.getTranslation()) * 100) / 100.0;
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
        savedDataCountPublisher.set(savedShooterDistanceVelocityMap.size());
    }

    private Translation2d getTarget() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return FieldConstants.HUB_RED;
        }
        return FieldConstants.HUB_BLUE;
    }

    private double calculateAngularVelocity(double distanceToTarget) {
        return switch (calculationMode) {
            case INTERPOLATION -> Objects.requireNonNullElse(shooterDistanceVelocityMap.get(distanceToTarget), 0.0);
            case EQUATION -> compiledExpression.evaluate(distanceToTarget);
        };
    }

    private String exportData() {
        final var entries = savedShooterDistanceVelocityMap.entrySet();
        if (entries.isEmpty()) return NO_DATA_TEXT;
        return entries.stream()
                .map(e -> e.getKey() + "," + e.getValue())
                .reduce("", (a, b) -> (a.isEmpty() ? "" : ";") + b);
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

        if (exportToConsoleButton.get()) {
            // button pressed
            exportToConsoleButton.set(false);
            System.out.println("EXPORT: " + exportData());
        }

        final var currentDashboardData = savedShooterDistanceVelocityMapEntry.get();
        final var exportedData = exportData();
        if (!currentDashboardData.equals(exportedData)) {
            importData();
        }

        if (!equationSubscriber.get().equals(originalExpression)) {
            originalExpression = equationSubscriber.get();
            compileEquation();
        }

        if (System.currentTimeMillis() - lastUpdateTimestampMillis >= 100L) {
            calculateVelocity();
        }
    }

    private void compileEquation() {
        final var env = new EvaluationEnvironment();
        env.setVariableNames("x");
        final var rawExpression = equationSubscriber.get();
        CompiledExpression compiled;
        try {
            compiled = Crunch.compileExpression(rawExpression, env);
        } catch (ExpressionCompilationException | ExpressionEvaluationException ex) {
            DriverStation.reportError("Error compiling equation for shooter: " + ex.getMessage(), ex.getStackTrace());
            compiled = Crunch.compileExpression("0");
        }
        originalExpression = rawExpression;
        this.compiledExpression = compiled;
    }

    public Command increaseVelocityOffset() {
        return adjustVelocityOffset(true);
    }

    public Command decreaseVelocityOffset() {
        return adjustVelocityOffset(false);
    }

    private Command adjustVelocityOffset(boolean increase) {
        return Commands.run(() -> {
            final var velocityChange =
                    RotationsPerSecondPerSecond.of(6).times(Milliseconds.of(20)).in(RotationsPerSecond);
            velocityOffsetEntry.set(velocityOffsetEntry.get() + (increase ? velocityChange : -velocityChange));
        });
    }

    public record ShotData(Distance distance, AngularVelocity velocity, Rotation2d heading) {}

    private enum CalculationMode {
        INTERPOLATION,
        EQUATION
    }
}
