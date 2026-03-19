package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.dashboard.LoggedNetworkDouble;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.FieldConstants;
import frc.robot.util.enums.Constants.PhysicalConstants;
import frc.robot.util.enums.Constants.ShootOnTheMoveConstants;
import frc.robot.util.lib.frcfirecontrol.FuelPhysicsSim;
import frc.robot.util.lib.frcfirecontrol.ProjectileSimulator;
import frc.robot.util.lib.frcfirecontrol.ShotCalculator;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.ThreadLocalRandom;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import redempt.crunch.CompiledExpression;
import redempt.crunch.Crunch;
import redempt.crunch.exceptions.ExpressionCompilationException;
import redempt.crunch.exceptions.ExpressionEvaluationException;
import redempt.crunch.functional.EvaluationEnvironment;
import swervelib.SwerveDrive;

// Credits to
// https://github.com/FRCTeam360/RainMaker26/blob/main/src/main/java/frc/robot/subsystems/Shooter/ShotCalculator.java
@NullMarked
public class ShooterCalculator {
    private final SwerveDrive swerveDrive;
    private final InterpolatingDoubleTreeMap shooterDistanceVelocityMap = new InterpolatingDoubleTreeMap();
    private final Map<Double, Double> savedShooterDistanceVelocityMap =
            new TreeMap<>(); // because InterpolatingDoubleTreeMap won't let us extract its values
    private @Nullable ShotData lastShotData;
    private CalculationMode calculationMode = CalculationMode.SOTM;
    private boolean manualMode = true;
    private String originalExpression = "-45.66 * x^(-2.622) + 56.778";
    private CompiledExpression compiledExpression = Crunch.compileExpression("0");
    private long lastUpdateTimestampMillis;
    private ShotCalculator shotCalculator;
    private ProjectileSimulator sotmSimulator;
    private ShotCalculator.@Nullable LaunchParameters lastSOTMLaunchParameters;

    // Sim
    private final FuelPhysicsSim fuelPhysicsSim = new FuelPhysicsSim("Sim/FuelPositions");
    private BooleanSupplier isIntaking = () -> false;
    private BooleanSupplier isLaunching = () -> false;
    private long simLastLaunchTime = 0L;
    private int simBallsInHopper = 0;
    private final IntegerPublisher simBallsInHopperPublisher;

    private final BooleanEntry manualModeEntry;
    private final StringPublisher manualModeTextPublisher;
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
    private final DoublePublisher shotConfidencePublisher;
    private final LoggedNetworkUnit<AngleUnit, Angle> loggedLaunchAngle;
    private final LoggedNetworkDouble loggedSlipFactor;

    private static final String NO_DATA_TEXT = "(No Data)";

    public ShooterCalculator(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        final var topicPrefix = "Shooter Calculator/";
        manualModeEntry = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Manual Mode")
                .getEntry(manualMode);
        manualModeEntry.set(manualMode);
        manualModeTextPublisher = NetworkTableInstance.getDefault()
                .getStringTopic("Manual Mode Text")
                .publish();
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
        shotConfidencePublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Shot Confidence")
                .publish();
        loggedLaunchAngle = new LoggedNetworkUnit<>(
                topicPrefix + "Launch Angle", ShootOnTheMoveConstants.LAUNCH_ANGLE_FROM_HORIZONTAL);
        loggedLaunchAngle.addListener(unused -> shotCalculator = createShotCalculator());
        loggedSlipFactor = new LoggedNetworkDouble(topicPrefix + "Slip Factor", ShootOnTheMoveConstants.SLIP_FACTOR);
        loggedSlipFactor.addListener(unused -> shotCalculator = createShotCalculator());

        simBallsInHopperPublisher = NetworkTableInstance.getDefault()
                .getIntegerTopic("Sim/Balls in Hopper")
                .publish();
        simBallsInHopperPublisher.set(simBallsInHopper);

        sotmSimulator = createProjectileSimulator();
        shotCalculator = createShotCalculator();

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

    public boolean isUsingSOTM() {
        return !isManualModeEnabled() && calculationMode == CalculationMode.SOTM;
    }

    private ShotCalculator.LaunchParameters calculateSOTM() {
        if (lastSOTMLaunchParameters != null) {
            cachedPublisher.set(true);
            return lastSOTMLaunchParameters;
        }
        cachedPublisher.set(false);
        lastUpdateTimestampMillis = System.currentTimeMillis();
        final var shotInputs = new ShotCalculator.ShotInputs(
                swerveDrive.getPose(),
                swerveDrive.getFieldVelocity(),
                swerveDrive.getRobotVelocity(),
                getTarget(),
                getTargetForwardVector(),
                0.9, // vision confidence, from 0 to 1
                swerveDrive.getPitch().getDegrees(),
                swerveDrive.getRoll().getDegrees());
        final var shot = shotCalculator.calculate(shotInputs);
        // System.out.println("\n\nShot: " + shot + "\n\nrpm map: ");
        shotConfidencePublisher.set(shot.confidence());
        lastSOTMLaunchParameters = shot;
        return shot;
    }

    public ShotData calculateShotData() {
        if (lastShotData != null) {
            cachedPublisher.set(true);
            return lastShotData;
        }
        cachedPublisher.set(false);
        lastUpdateTimestampMillis = System.currentTimeMillis();
        final Pose2d robotPose = swerveDrive.getPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d target = getTarget();
        final double distanceToTarget = target.getDistance(robotTranslation);

        final double targetVelocity = calculateAngularVelocity(distanceToTarget) + velocityOffsetEntry.get();
        final var sotmData = (calculationMode == CalculationMode.SOTM) ? calculateSOTM() : null;
        final Rotation2d targetHeading = (sotmData != null
                ? sotmData.driveAngle().rotateBy(Rotation2d.k180deg)
                : target.minus(robotTranslation).getAngle());
        final var distance = Meters.of(sotmData != null ? sotmData.solvedDistanceM() : distanceToTarget);
        final var shooterVelocity = sotmData != null ? RPM.of(sotmData.rpm()) : RotationsPerSecond.of(targetVelocity);
        final var driveAngleFF = RadiansPerSecond.of(sotmData != null ? sotmData.driveAngularVelocityRadPerSec() : 0);
        final var isReady = sotmData != null
                ? (sotmData.isValid() && sotmData.confidence() > 50)
                : (Math.abs(robotPose.getRotation().minus(targetHeading).getDegrees()) < 10);
        final var shot = new ShotData(distance, shooterVelocity, targetHeading, driveAngleFF, isReady);
        shotDistancePublisher.set(shot.distance().in(Meters));
        shotVelocityPublisher.set(shot.velocity().in(RotationsPerSecond));
        shotHeadingPublisher.set(shot.heading().getDegrees());
        invertedShotHeadingPublisher.set(-(shot.heading.getDegrees() + 180));
        lastShotData = shot;
        return shot;
    }

    public void addCurrentDataToMap(AngularVelocity shooterVelocity) {
        final Pose2d robotPose = swerveDrive.getPose();
        final double distanceToTarget = Math.round(getTarget().getDistance(robotPose.getTranslation()) * 100) / 100.0;
        addRawDistanceVelocityData(distanceToTarget, shooterVelocity.in(RotationsPerSecond));
    }

    private ProjectileSimulator createProjectileSimulator() {
        final var sotmParams = new ProjectileSimulator.SimParameters(
                ShootOnTheMoveConstants.BALL_MASS.in(Kilograms),
                ShootOnTheMoveConstants.BALL_DIAMETER.in(Meters),
                ShootOnTheMoveConstants.DRAG_COEFFICIENT,
                ShootOnTheMoveConstants.MAGNUS_COEFFICIENT,
                ShootOnTheMoveConstants.AIR_DENSITY, // kg/m³
                ShootOnTheMoveConstants.EXIT_HEIGHT.in(Meters),
                ShootOnTheMoveConstants.FLYWHEEL_DIAMETER.in(Meters),
                ShootOnTheMoveConstants.HUB_HEIGHT.in(Meters),
                loggedSlipFactor.getAsDouble(),
                loggedLaunchAngle.get().in(Degrees),
                ShootOnTheMoveConstants.SIM_TIMESTEP.in(Seconds),
                ShootOnTheMoveConstants.RPM_SEARCH_MIN.in(RPM),
                ShootOnTheMoveConstants.RPM_SEARCH_MAX.in(RPM),
                ShootOnTheMoveConstants.ITERATIONS,
                ShootOnTheMoveConstants.MAX_SIM_TIME.in(Seconds));
        return new ProjectileSimulator(sotmParams);
    }

    private ShotCalculator createShotCalculator() {
        sotmSimulator = createProjectileSimulator();
        final var lut = sotmSimulator.generateLUT();
        final var shotCalcConfig = new ShotCalculator.Config();
        shotCalcConfig.launcherOffsetX = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getX();
        shotCalcConfig.launcherOffsetY = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getY();
        shotCalcConfig.phaseDelayMs = ShootOnTheMoveConstants.PHASE_DELAY.in(Milliseconds);
        shotCalcConfig.mechLatencyMs = ShootOnTheMoveConstants.MECHANISM_LATENCY.in(Milliseconds);
        shotCalcConfig.maxTiltDeg = ShootOnTheMoveConstants.MAXIMUM_TILT.in(Degrees);
        shotCalcConfig.headingSpeedScalar = ShootOnTheMoveConstants.HEADING_SPEED_SCALAR;
        shotCalcConfig.headingReferenceDistance = ShootOnTheMoveConstants.HEADING_REFERENCE_DISTANCE;
        final var shotCalc = new ShotCalculator(shotCalcConfig);
        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
        return shotCalc;
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

    private Translation2d getTargetForwardVector() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return ShootOnTheMoveConstants.RED_HUB_FORWARD_VECTOR;
        }
        return ShootOnTheMoveConstants.BLUE_HUB_FORWARD_VECTOR;
    }

    private double calculateAngularVelocity(double distanceToTarget) {
        return switch (calculationMode) {
            case INTERPOLATION -> Objects.requireNonNullElse(shooterDistanceVelocityMap.get(distanceToTarget), 0.0);
            case EQUATION -> compiledExpression.evaluate(distanceToTarget);
            case SOTM -> RotationsPerSecond.convertFrom(calculateSOTM().rpm(), RPM);
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

    public void simulationInit() {
        fuelPhysicsSim.enable();
        fuelPhysicsSim.placeFieldBalls();

        fuelPhysicsSim.configureRobot(
                PhysicalConstants.ROBOT_WIDTH_Y.in(Meters),
                PhysicalConstants.ROBOT_LENGTH_X.in(Meters),
                PhysicalConstants.BUMPERS_HEIGHT.in(Meters),
                swerveDrive::getPose,
                swerveDrive::getFieldVelocity);

        fuelPhysicsSim.addIntakeZone(
                Meters.convertFrom(4.4, Inches),
                Meters.convertFrom(16, Inches),
                Meters.convertFrom(-8, Inches),
                Meters.convertFrom(8, Inches),
                () -> isIntaking.getAsBoolean() && simBallsInHopper <= 20,
                () -> setSimBallsInHopper(simBallsInHopper + 1));
    }

    public void simulationPeriodic() {
        fuelPhysicsSim.tick();
        if (isLaunching.getAsBoolean()) {
            if (System.currentTimeMillis() < simLastLaunchTime) {
                return;
            }
            if (simBallsInHopper <= 0) {
                return;
            }
            simLastLaunchTime =
                    System.currentTimeMillis() + ThreadLocalRandom.current().nextLong(100, 400);
            setSimBallsInHopper(simBallsInHopper - 1);
            final var shotData = calculateShotData();
            final var rpm = shotData.velocity().in(RPM);
            final var ballSpeed = sotmSimulator.exitVelocity(rpm);
            fuelPhysicsSim.launchBall(
                    new Translation3d(swerveDrive.getPose().getTranslation())
                            .plus(new Translation3d(
                                            ShootOnTheMoveConstants.LAUNCHER_OFFSET.getMeasureX(),
                                            ShootOnTheMoveConstants.LAUNCHER_OFFSET.getMeasureY(),
                                            ShootOnTheMoveConstants.EXIT_HEIGHT)
                                    .rotateBy(swerveDrive.getGyroRotation3d())),
                    new Translation3d(
                            ballSpeed,
                            new Rotation3d(
                                            Degrees.zero(),
                                            ShootOnTheMoveConstants.LAUNCH_ANGLE_FROM_HORIZONTAL.plus(Degrees.of(180)),
                                            Degrees.zero())
                                    .rotateBy(swerveDrive.getGyroRotation3d())),
                    rpm);
        }
    }

    private void setSimBallsInHopper(int newBallsInHopper) {
        simBallsInHopper = newBallsInHopper;
        simBallsInHopperPublisher.set(simBallsInHopper);
    }

    public void prePeriodic() {
        lastShotData = null;
        lastSOTMLaunchParameters = null;

        // NetworkTables
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
        manualMode = manualModeEntry.get();
        manualModeTextPublisher.set(manualMode ? "Custom" : "Calculator");

        if (System.currentTimeMillis() - lastUpdateTimestampMillis >= 100L) {
            calculateShotData();
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

    private void setManualMode(boolean newMode) {
        manualMode = newMode;
        manualModeEntry.set(newMode);
    }

    public Command temporarilyEnableManualMode() {
        return Commands.deferredProxy(() -> {
            if (manualMode) return Commands.none(); // already enabled, so this wouldn't do anything
            return Commands.startEnd(() -> setManualMode(true), () -> setManualMode(false));
        });
    }

    public boolean isManualModeEnabled() {
        return manualMode;
    }

    public void setIsIntaking(BooleanSupplier isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void setIsLaunching(BooleanSupplier isLaunching) {
        this.isLaunching = isLaunching;
    }

    public record ShotData(
            Distance distance,
            AngularVelocity velocity,
            Rotation2d heading,
            AngularVelocity driveAngleFF,
            boolean isReady) {}

    private enum CalculationMode {
        INTERPOLATION,
        EQUATION,
        SOTM
    }
}
