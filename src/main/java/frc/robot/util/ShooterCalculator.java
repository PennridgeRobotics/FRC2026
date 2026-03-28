package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkButton;
import frc.robot.util.dashboard.LoggedNetworkDouble;
import frc.robot.util.dashboard.LoggedNetworkInteger;
import frc.robot.util.dashboard.LoggedNetworkSendable;
import frc.robot.util.dashboard.LoggedNetworkString;
import frc.robot.util.dashboard.LoggedNetworkStruct;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.FieldConstants;
import frc.robot.util.enums.Constants.PassingConstants;
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
    private @Nullable ShotData lastPassingData;
    private CalculationMode calculationMode = CalculationMode.SOTM;
    private final String defaultExpression = "-45.66 * x^(-2.622) + 56.778";
    private CompiledExpression compiledExpression = Crunch.compileExpression("0");
    private long lastUpdateTimestampMillis;
    private ShotCalculator shotCalculator;
    private ProjectileSimulator sotmSimulator;
    private ShotCalculator passingCalculator;
    private ProjectileSimulator passingSimulator;
    private ShotCalculator.@Nullable LaunchParameters lastSOTMLaunchParameters;
    private ShotCalculator.@Nullable LaunchParameters lastPassingLaunchParameters;
    private @Nullable AngularVelocity lastAngularVelocityInput;

    // Sim
    private final FuelPhysicsSim fuelPhysicsSim = new FuelPhysicsSim("Sim/FuelPositions");
    private BooleanSupplier isIntaking = () -> false;
    private BooleanSupplier isLaunching = () -> false;
    private long simLastLaunchTime = 0L;
    private final LoggedNetworkInteger loggedSimBallsInHopper;
    private final LoggedNetworkInteger loggedSimHopperLimit;
    private final LoggedNetworkUnit<TimeUnit, Time> loggedSimShootCooldownMinMs;
    private final LoggedNetworkUnit<TimeUnit, Time> loggedSimShootCooldownMaxMs;
    private final LoggedNetworkBoolean loggedSimToggleRemoveScoredBalls;
    private final LoggedNetworkStruct<Translation3d> loggedSimBallTargetPos;

    private final LoggedNetworkBoolean loggedManualMode;
    private final LoggedNetworkBoolean passingAllowed;
    private final LoggedNetworkBoolean currentlyPassing;
    private final LoggedNetworkUnit<DistanceUnit, Distance> loggedShotDistance;
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> loggedShotVelocity;
    private final LoggedNetworkUnit<AngleUnit, Angle> loggedShotHeading;
    private final LoggedNetworkUnit<AngleUnit, Angle> loggedInvertedShotHeading;
    private final LoggedNetworkString loggedSavedShooterDistanceVelocityMap;
    private final LoggedNetworkString loggedEquation;
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> loggedVelocityOffset;
    private final LoggedNetworkDouble loggedShotConfidence;
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> loggedDriveAngleFF;
    private final LoggedNetworkUnit<AngleUnit, Angle> loggedLaunchAngle;
    private final LoggedNetworkDouble loggedSlipFactor;
    private final LoggedNetworkUnit<TimeUnit, Time> loggedPhaseDelay;
    private final LoggedNetworkUnit<TimeUnit, Time> loggedMechanismDelay;
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> loggedManualLaunchVelocity;

    private static final String NO_DATA_TEXT = "(No Data)";

    public ShooterCalculator(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        final var topicPrefix = "Shooter Calculator/";
        final var rootTopicPrefix = "/" + topicPrefix;
        loggedManualMode = new LoggedNetworkBoolean(topicPrefix + "Manual Mode", false);
        new LoggedNetworkString(
                rootTopicPrefix + "Manual Mode Text", () -> loggedManualMode.getAsBoolean() ? "Custom" : "Calculator");
        passingAllowed = new LoggedNetworkBoolean(rootTopicPrefix + "Passing Allowed", true);
        currentlyPassing = new LoggedNetworkBoolean(rootTopicPrefix + "Passing", this::shouldBePassing);
        loggedShotDistance = new LoggedNetworkUnit<>(rootTopicPrefix + "Shot Distance", Meters.zero());
        loggedShotVelocity = new LoggedNetworkUnit<>(rootTopicPrefix + "Shot Velocity", RotationsPerSecond.zero());
        loggedShotHeading = new LoggedNetworkUnit<>(rootTopicPrefix + "Shot Heading", Degrees.zero());
        loggedInvertedShotHeading = new LoggedNetworkUnit<>(rootTopicPrefix + "Inverted Shot Heading", Degrees.zero());
        new LoggedNetworkStruct<Pose2d>(rootTopicPrefix + "Shot Heading Pose", Pose2d.struct, () -> {
            final var pose = swerveDrive.getPose();
            final var heading = Rotation2d.fromRadians(loggedShotHeading.get().in(Radians));
            return new Pose2d(pose.getTranslation(), heading).plus(new Transform2d(-100.0, 0, Rotation2d.kZero));
        });
        new LoggedNetworkInteger(rootTopicPrefix + "Saved Data Count", savedShooterDistanceVelocityMap::size);
        loggedSavedShooterDistanceVelocityMap =
                new LoggedNetworkString(topicPrefix + "Saved Shooter Distance Velocity Map", NO_DATA_TEXT);
        loggedSavedShooterDistanceVelocityMap.addListener(unused -> importData());
        new LoggedNetworkButton(topicPrefix + "Save Current Distance and Velocity", () -> {
            if (lastShotData == null) return;
            addDistanceVelocityData(lastShotData.distance(), lastShotData.velocity());
        });
        new LoggedNetworkButton(topicPrefix + "Export to Console", () -> System.out.println("EXPORT: " + exportData()));
        loggedEquation = new LoggedNetworkString(topicPrefix + "Equation", defaultExpression);
        loggedEquation.addListener(unused -> compileEquation());
        compileEquation();
        loggedVelocityOffset = new LoggedNetworkUnit<>(topicPrefix + "Velocity Offset", RotationsPerSecond.zero());
        loggedShotConfidence = new LoggedNetworkDouble(rootTopicPrefix + "Shot Confidence", 0.0);
        loggedDriveAngleFF = new LoggedNetworkUnit<>(rootTopicPrefix + "Drive Angle FF", DegreesPerSecond.zero());
        loggedLaunchAngle = new LoggedNetworkUnit<>(
                topicPrefix + "Launch Angle", ShootOnTheMoveConstants.LAUNCH_ANGLE_FROM_HORIZONTAL);
        loggedLaunchAngle.addListener(unused -> {
            shotCalculator = createSOTMShotCalculator();
            passingCalculator = createPassingShotCalculator();
        });
        loggedSlipFactor = new LoggedNetworkDouble(topicPrefix + "Slip Factor", ShootOnTheMoveConstants.SLIP_FACTOR);
        loggedSlipFactor.addListener(unused -> {
            shotCalculator = createSOTMShotCalculator();
            passingCalculator = createPassingShotCalculator();
        });
        loggedPhaseDelay = new LoggedNetworkUnit<>(topicPrefix + "Phase Delay", ShootOnTheMoveConstants.PHASE_DELAY);
        loggedPhaseDelay.addListener(unused -> {
            shotCalculator = createSOTMShotCalculator();
            passingCalculator = createPassingShotCalculator();
        });
        loggedMechanismDelay =
                new LoggedNetworkUnit<>(topicPrefix + "Mechanism Delay", ShootOnTheMoveConstants.MECHANISM_LATENCY);
        loggedMechanismDelay.addListener(unused -> {
            shotCalculator = createSOTMShotCalculator();
            passingCalculator = createPassingShotCalculator();
        });
        loggedManualLaunchVelocity =
                new LoggedNetworkUnit<>(topicPrefix + "Manual Launch Velocity", RotationsPerSecond.of(47));
        new LoggedNetworkSendable<>(
                topicPrefix + "Calculation Mode",
                SplitButtonChooser.withEnum(
                        () -> calculationMode,
                        Set.of(newMode -> calculationMode = newMode),
                        calculationMode,
                        CalculationMode.class));

        loggedSimBallsInHopper = new LoggedNetworkInteger("/Sim/Balls in Hopper", 1000);
        loggedSimHopperLimit = new LoggedNetworkInteger("/Sim/Hopper Limit", 20);
        loggedSimShootCooldownMinMs = new LoggedNetworkUnit<>("/Sim/Shoot Cooldown Min MS", Milliseconds.of(80));
        loggedSimShootCooldownMaxMs = new LoggedNetworkUnit<>("/Sim/Shoot Cooldown Max MS", Milliseconds.of(100));
        new LoggedNetworkButton("/Sim/Clear Balls", fuelPhysicsSim::clearBalls);
        new LoggedNetworkButton("/Sim/Spawn Balls", fuelPhysicsSim::placeFieldBalls);
        loggedSimToggleRemoveScoredBalls = new LoggedNetworkBoolean("/Sim/Remove Scored Balls", true);
        loggedSimToggleRemoveScoredBalls.addListener(fuelPhysicsSim::setRemoveScoredBalls);
        loggedSimBallTargetPos =
                new LoggedNetworkStruct<>("/Sim Ball Target Pos", Translation3d.struct, new Translation3d());

        sotmSimulator = createProjectileSimulator(ShootOnTheMoveConstants.HUB_HEIGHT);
        shotCalculator = createSOTMShotCalculator();

        passingSimulator = createProjectileSimulator(Inches.zero());
        passingCalculator = createPassingShotCalculator();

        addPreviouslySavedData();
    }

    private void addPreviouslySavedData() {
        // Generated from https://docs.google.com/spreadsheets/d/1CAVIv4i_sHdjmRlEvcaaNroXZzFWiLivQj3PQpYls7k

    }

    public boolean isUsingSOTM() {
        return !isManualModeEnabled() && calculationMode == CalculationMode.SOTM;
    }

    private ShotCalculator.LaunchParameters calculateSOTM(boolean passing) {
        if (!passing) {
            if (lastSOTMLaunchParameters != null) {
                return lastSOTMLaunchParameters;
            }
        } else if (lastPassingLaunchParameters != null) {
            return lastPassingLaunchParameters;
        }
        lastUpdateTimestampMillis = System.currentTimeMillis();
        final var shotInputs = new ShotCalculator.ShotInputs(
                swerveDrive.getPose(),
                withInputAngularVelocity(swerveDrive.getFieldVelocity()),
                withInputAngularVelocity(swerveDrive.getRobotVelocity()),
                passing ? getPassingTarget() : getHubTranslation(),
                passing ? getPassingForwardVector() : getHubForwardVector(),
                0.9, // vision confidence, from 0 to 1
                swerveDrive.getPitch().getDegrees(),
                swerveDrive.getRoll().getDegrees());
        final var shot = (passing ? passingCalculator : shotCalculator).calculate(shotInputs);
        // System.out.println("\n\nShot: " + shot + "\n\nrpm map: ");
        loggedShotConfidence.set(shot.confidence());
        loggedDriveAngleFF.set(RadiansPerSecond.of(shot.driveAngularVelocityRadPerSec()));
        lastSOTMLaunchParameters = shot;
        return shot;
    }

    private ChassisSpeeds withInputAngularVelocity(ChassisSpeeds chassisSpeeds) {
        if (lastAngularVelocityInput == null) return chassisSpeeds;
        return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                lastAngularVelocityInput.in(RadiansPerSecond));
    }

    public ShotData calculateShotData() {
        return calculateShotData(shouldBePassing());
    }

    public ShotData calculateShotData(boolean passing) {
        if (!passing) {
            if (lastShotData != null) {
                return lastShotData;
            }
        } else if (lastPassingData != null) {
            return lastPassingData;
        }
        lastUpdateTimestampMillis = System.currentTimeMillis();
        final Pose2d robotPose = swerveDrive.getPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d target = passing ? getPassingTarget() : getHubTranslation();
        final double distanceToTarget = target.getDistance(robotTranslation);

        final double targetVelocity = calculateAngularVelocity(distanceToTarget, passing);
        final var sotmData = isUsingSOTM() ? calculateSOTM(passing) : null;
        final Rotation2d targetHeading = (sotmData != null
                ? sotmData.driveAngle()
                : target.minus(robotTranslation).getAngle().rotateBy(Rotation2d.k180deg));
        final var distance = Meters.of(distanceToTarget);
        final var shooterVelocity = RotationsPerSecond.of(targetVelocity);
        final var driveAngleFF = RadiansPerSecond.of(sotmData != null ? sotmData.driveAngularVelocityRadPerSec() : 0);
        final var isReady = sotmData != null
                ? (sotmData.isValid() && sotmData.confidence() > (passing ? 35 : 50))
                : (Math.abs(robotPose.getRotation().minus(targetHeading).getDegrees()) < 10);
        final var shot = new ShotData(distance, shooterVelocity, targetHeading, driveAngleFF, isReady);
        loggedShotDistance.set(shot.distance());
        loggedShotVelocity.set(shot.velocity());
        loggedShotHeading.set(shot.heading().getMeasure());
        loggedInvertedShotHeading.set(
                shot.heading().getMeasure().plus(Degrees.of(180)).unaryMinus());
        lastShotData = shot;
        return shot;
    }

    public void addCurrentDataToMap(AngularVelocity shooterVelocity) {
        final Pose2d robotPose = swerveDrive.getPose();
        final double distanceToTarget =
                Math.round(getHubTranslation().getDistance(robotPose.getTranslation()) * 100) / 100.0;
        addRawDistanceVelocityData(distanceToTarget, shooterVelocity.in(RotationsPerSecond));
    }

    private ProjectileSimulator createProjectileSimulator(Distance targetHeight) {
        final var sotmParams = new ProjectileSimulator.SimParameters(
                ShootOnTheMoveConstants.BALL_MASS.in(Kilograms),
                ShootOnTheMoveConstants.BALL_DIAMETER.in(Meters),
                ShootOnTheMoveConstants.DRAG_COEFFICIENT,
                ShootOnTheMoveConstants.MAGNUS_COEFFICIENT,
                ShootOnTheMoveConstants.AIR_DENSITY, // kg/m³
                ShootOnTheMoveConstants.EXIT_HEIGHT.in(Meters),
                ShootOnTheMoveConstants.FLYWHEEL_DIAMETER.in(Meters),
                targetHeight.in(Meters),
                loggedSlipFactor.getAsDouble(),
                loggedLaunchAngle.get().in(Degrees),
                ShootOnTheMoveConstants.SIM_TIMESTEP.in(Seconds),
                ShootOnTheMoveConstants.RPM_SEARCH_MIN.in(RPM),
                ShootOnTheMoveConstants.RPM_SEARCH_MAX.in(RPM),
                ShootOnTheMoveConstants.ITERATIONS,
                ShootOnTheMoveConstants.MAX_SIM_TIME.in(Seconds));
        return new ProjectileSimulator(sotmParams, ShootOnTheMoveConstants.MAGNUS_SIGN);
    }

    private ShotCalculator createSOTMShotCalculator() {
        sotmSimulator = createProjectileSimulator(ShootOnTheMoveConstants.HUB_HEIGHT);
        final var calculator = createShotCalculator(
                sotmSimulator,
                ShootOnTheMoveConstants.HEADING_SPEED_SCALAR,
                ShootOnTheMoveConstants.HEADING_REFERENCE_DISTANCE);
        final var tests = Map.of(
                2.0, 47.0,
                2.5, 50.0,
                3.2, 52.8,
                4.0, 58.0);
        double totalError = 0.0;
        for (var entry : tests.entrySet()) {
            final var distance = entry.getKey();
            final var velocity = entry.getValue();
            final var percentError = Math.abs(calculator.getBaseRPM(distance) / 60.0 - velocity) / velocity;
            totalError += percentError;
            System.out.printf(
                    "Expected for %.1fm: %.1f; got %.1f (%.1f%% error)\n",
                    distance, velocity, calculator.getBaseRPM(distance) / 60.0, percentError * 100);
        }
        System.out.printf("Average error: %.1f%%\n", totalError / tests.size() * 100);
        return calculator;
    }

    private ShotCalculator createShotCalculator(
            ProjectileSimulator projectileSimulator, double headingSpeedScalar, double headingReferenceDistance) {
        final var lut = projectileSimulator.generateLUT();
        final var shotCalcConfig = new ShotCalculator.Config();
        shotCalcConfig.launcherOffsetX = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getX();
        shotCalcConfig.launcherOffsetY = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getY();
        shotCalcConfig.phaseDelayMs =
                RobotBase.isReal() ? loggedPhaseDelay.get().in(Milliseconds) : 0;
        shotCalcConfig.mechLatencyMs =
                RobotBase.isReal() ? loggedMechanismDelay.get().in(Milliseconds) : 0;
        shotCalcConfig.maxTiltDeg = ShootOnTheMoveConstants.MAXIMUM_TILT.in(Degrees);
        shotCalcConfig.headingSpeedScalar = headingSpeedScalar;
        shotCalcConfig.headingReferenceDistance = headingReferenceDistance;
        shotCalcConfig.shooterAngleOffsetRad = Math.PI;
        shotCalcConfig.maxScoringDistance = PassingConstants.MAXIMUM_DISTANCE_PASSING.in(Meters);
        final var shotCalc = new ShotCalculator(shotCalcConfig);
        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
        return shotCalc;
    }

    private ShotCalculator createPassingShotCalculator() {
        passingSimulator = createProjectileSimulator(Inches.zero());
        return createShotCalculator(
                sotmSimulator,
                PassingConstants.HEADING_SPEED_SCALAR_PASSING,
                PassingConstants.HEADING_REFERENCE_DISTANCE_PASSING);
    }

    private void addDistanceVelocityData(Distance distance, AngularVelocity velocity) {
        addRawDistanceVelocityData(distance.in(Meters), velocity.in(RotationsPerSecond));
    }

    private void addRawDistanceVelocityData(double distance, double velocity) {
        System.out.println("Added distance " + distance + "m with velocity " + velocity + "rot/s");
        shooterDistanceVelocityMap.put(distance, velocity);
        savedShooterDistanceVelocityMap.put(distance, velocity);
        loggedSavedShooterDistanceVelocityMap.set(exportData());
    }

    private Translation2d getHubTranslation() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return FieldConstants.HUB_RED;
        }
        return FieldConstants.HUB_BLUE;
    }

    private Translation2d getHubForwardVector() {
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            return PassingConstants.RED_HUB_FORWARD_VECTOR;
        }
        return PassingConstants.BLUE_HUB_FORWARD_VECTOR;
    }

    private Translation2d getPassingForwardVector() {
        return getHubForwardVector().times(-1);
    }

    private double calculateAngularVelocity(double distanceToTarget, boolean passing) {
        if (loggedManualMode.getAsBoolean()) {
            return loggedManualLaunchVelocity.get().in(RotationsPerSecond);
        }
        return switch (calculationMode) {
                    case INTERPOLATION ->
                        Objects.requireNonNullElse(shooterDistanceVelocityMap.get(distanceToTarget), 0.0);
                    case EQUATION -> compiledExpression.evaluate(distanceToTarget);
                    case SOTM ->
                        RotationsPerSecond.convertFrom(calculateSOTM(passing).rpm(), RPM);
                }
                + loggedVelocityOffset.get().in(RotationsPerSecond);
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
        final var data = loggedSavedShooterDistanceVelocityMap.get();
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
        // fuelPhysicsSim.placeFieldBalls();

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
                () -> isIntaking.getAsBoolean() && loggedSimBallsInHopper.getAsInt() <= loggedSimHopperLimit.getAsInt(),
                () -> loggedSimBallsInHopper.set(loggedSimBallsInHopper.getAsInt() + 1));
        fuelPhysicsSim.setRemoveScoredBalls(loggedSimToggleRemoveScoredBalls.getAsBoolean());
    }

    public void simulationPeriodic() {
        fuelPhysicsSim.tick();

        loggedSimBallTargetPos.set(new Translation3d(
                FieldConstants.HUB_RED.getMeasureX(),
                FieldConstants.HUB_RED.getMeasureY(),
                ShootOnTheMoveConstants.HUB_HEIGHT));

        if (!isLaunching.getAsBoolean()) {
            return;
        }
        if (System.currentTimeMillis() < simLastLaunchTime) {
            return;
        }
        if (loggedSimBallsInHopper.getAsInt() <= 0) {
            return;
        }
        final var minShootCooldown = loggedSimShootCooldownMinMs.get().in(Milliseconds);
        final var maxShootCooldown = loggedSimShootCooldownMaxMs.get().in(Milliseconds);
        simLastLaunchTime = System.currentTimeMillis()
                + (maxShootCooldown <= minShootCooldown
                        ? Math.round(Math.max(minShootCooldown, maxShootCooldown))
                        : ThreadLocalRandom.current()
                                .nextLong(
                                        Math.round(loggedSimShootCooldownMinMs
                                                .get()
                                                .in(Milliseconds)),
                                        Math.round(loggedSimShootCooldownMaxMs
                                                .get()
                                                .in(Milliseconds))));
        loggedSimBallsInHopper.set(loggedSimBallsInHopper.getAsInt() - 1);
        final var shotData = calculateShotData(false);
        final var rpm = shotData.velocity().in(RPM);
        final var ballSpeed = sotmSimulator.exitVelocity(rpm);

        // ball spin
        final var ballSpinRadPerSec =
                ballSpeed / ShootOnTheMoveConstants.BALL_DIAMETER.div(2).in(Meters);
        final var ballSpinVector =
                new Translation3d(0.0, ballSpinRadPerSec, 0.0).rotateBy(swerveDrive.getGyroRotation3d());

        final var launchPosition = new Translation3d(swerveDrive.getPose().getTranslation())
                .plus(new Translation3d(
                                ShootOnTheMoveConstants.LAUNCHER_OFFSET.getMeasureX(),
                                ShootOnTheMoveConstants.LAUNCHER_OFFSET.getMeasureY(),
                                ShootOnTheMoveConstants.EXIT_HEIGHT)
                        .rotateBy(swerveDrive.getGyroRotation3d()));

        final var launchAngleRadians = loggedLaunchAngle.get().in(Radians);
        final var ballVelocityWhileStill = new Translation3d(
                        -ballSpeed * Math.cos(launchAngleRadians), 0.0, ballSpeed * Math.sin(launchAngleRadians))
                .rotateBy(swerveDrive.getGyroRotation3d());

        // see ShotCalculator#calculate
        final var cosH = swerveDrive.getPose().getRotation().getCos();
        final var sinH = swerveDrive.getPose().getRotation().getSin();
        final var launcherFieldOffX = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getX() * cosH
                - ShootOnTheMoveConstants.LAUNCHER_OFFSET.getY() * sinH;
        final var launcherFieldOffY = ShootOnTheMoveConstants.LAUNCHER_OFFSET.getX() * sinH
                + ShootOnTheMoveConstants.LAUNCHER_OFFSET.getY() * cosH;
        final var omega = swerveDrive.getGyro().getYawAngularVelocity().in(RadiansPerSecond);
        final var vx = swerveDrive.getFieldVelocity().vxMetersPerSecond + (-launcherFieldOffY) * omega;
        final var vy = swerveDrive.getFieldVelocity().vyMetersPerSecond + launcherFieldOffX * omega;
        final var ballVelocity = ballVelocityWhileStill.plus(new Translation3d(vx, vy, 0.0));

        fuelPhysicsSim.launchBall(launchPosition, ballVelocity, ballSpinVector);
    }

    public void prePeriodic() {
        lastShotData = null;
        lastSOTMLaunchParameters = null;
        lastPassingLaunchParameters = null;

        if (System.currentTimeMillis() - lastUpdateTimestampMillis >= 100L) {
            calculateShotData(false);
            calculateShotData(true);
        }
    }

    private void compileEquation() {
        final var env = new EvaluationEnvironment();
        env.setVariableNames("x");
        final var rawExpression = loggedEquation.get();
        CompiledExpression compiled;
        try {
            compiled = Crunch.compileExpression(rawExpression, env);
        } catch (ExpressionCompilationException | ExpressionEvaluationException ex) {
            DriverStation.reportError("Error compiling equation for shooter: " + ex.getMessage(), ex.getStackTrace());
            compiled = Crunch.compileExpression("0");
        }
        this.compiledExpression = compiled;
    }

    public boolean shouldBePassing() {
        if (!passingAllowed.getAsBoolean()) return false;
        final var isRed = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        final var currentX = swerveDrive.getPose().getMeasureX();
        final var neededX = getHubTranslation().getMeasureX();
        return isRed ? currentX.lt(neededX) : currentX.gt(neededX);
    }

    public Translation2d getPassingTarget() {
        final var isRed = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        final var isOnLeftSide = isRed == swerveDrive.getPose().getMeasureY().lt(FieldConstants.FIELD_WIDTH_Y.div(2));
        final Translation2d target;
        if (isOnLeftSide) {
            target = isRed ? PassingConstants.PASSING_SPOT_LEFT_RED : PassingConstants.PASSING_SPOT_LEFT_BLUE;
        } else {
            target = isRed ? PassingConstants.PASSING_SPOT_RIGHT_RED : PassingConstants.PASSING_SPOT_RIGHT_BLUE;
        }
        if (isManualModeEnabled()) {
            return new Translation2d(target.getMeasureX(), swerveDrive.getPose().getMeasureY());
        }
        return target;
    }

    public Command increaseVelocityOffset() {
        return adjustVelocityOffset(true);
    }

    public Command decreaseVelocityOffset() {
        return adjustVelocityOffset(false);
    }

    private Command adjustVelocityOffset(boolean increase) {
        return Commands.run(() -> {
            final var velocityChange = RotationsPerSecondPerSecond.of(6).times(Milliseconds.of(20));
            loggedVelocityOffset.set(
                    loggedVelocityOffset.get().plus(increase ? velocityChange : velocityChange.unaryMinus()));
        });
    }

    public Command increaseManualLaunchVelocity() {
        return adjustManualLaunchVelocity(true);
    }

    public Command decreaseManualLaunchVelocity() {
        return adjustManualLaunchVelocity(false);
    }

    private Command adjustManualLaunchVelocity(boolean increase) {
        return Commands.run(() -> {
            final var velocityChange = RotationsPerSecondPerSecond.of(6).times(Milliseconds.of(20));
            loggedManualLaunchVelocity.set(
                    loggedManualLaunchVelocity.get().plus(increase ? velocityChange : velocityChange.unaryMinus()));
        });
    }

    public Command temporarilyEnableManualMode() {
        return Commands.deferredProxy(() -> {
            if (loggedManualMode.getAsBoolean())
                return Commands.none(); // already enabled, so this wouldn't do anything
            return Commands.startEnd(() -> loggedManualMode.set(true), () -> loggedManualMode.set(false));
        });
    }

    public boolean isManualModeEnabled() {
        return loggedManualMode.getAsBoolean();
    }

    public void setIsIntaking(BooleanSupplier isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void setIsLaunching(BooleanSupplier isLaunching) {
        this.isLaunching = isLaunching;
    }

    public void setLastAngularVelocityInput(@Nullable AngularVelocity lastAngularVelocityInput) {
        this.lastAngularVelocityInput = lastAngularVelocityInput;
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
