package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.dashboard.LoggedNetworkStruct;
import frc.robot.util.dashboard.LoggedNetworkStructArray;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.enums.Constants.FieldConstants;
import frc.robot.util.enums.Constants.PhysicalConstants;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class AutoManager {
    private final SwerveSubsystem swerveDrive;
    private final FollowPath.Builder pathBuilder;
    private final FuelSubsystem fuelSubsystem;
    private final @Nullable ClimberSubsystem climberSubsystem;

    // Built-in paths
    private final Path startLeftHubShootPath = new Path("start_left_hub_shoot");
    private final Path startLeftInnerBumpShootPath = new Path("start_left_inner_bump_shoot");
    private final Path startRightInnerBumpShootPath = new Path("start_right_inner_bump_shoot");
    private final Path toOutpostPath = new Path("to_outpost");
    private final Path alignClimbPath = new Path("align_climb");
    private final Path depotPath = new Path("depot");
    private final Path collectMidFromLeftPath = new Path("collect_mid_from_left");
    private final Path collectMidFromRightPath = new Path("collect_mid_from_right");
    private final Supplier<Distance> distanceSupplier =
            new LoggedNetworkUnit<>("Auto/Move from hub & shoot distance (m)", Meters.of(1.8));

    private @Nullable Pair<FollowPath, Path> currentPath;
    private @Nullable Pose2d pathStart;

    private final LoggedNetworkUnit<DistanceUnit, Distance> testAutoX =
            new LoggedNetworkUnit<>("Auto/Test X", Meters.of(2));
    private final LoggedNetworkUnit<DistanceUnit, Distance> testAutoY =
            new LoggedNetworkUnit<>("Auto/Test Y", Meters.of(0));
    private final LoggedNetworkUnit<AngleUnit, Angle> testAutoAngle =
            new LoggedNetworkUnit<>("Auto/Test Angle", Degrees.of(0));
    private final LoggedNetworkUnit<LinearVelocityUnit, LinearVelocity> testAutoLeadInVelocity =
            new LoggedNetworkUnit<>("Auto/Test Lead in Velocity", MetersPerSecond.of(1));

    private final @Nullable Field pathField;

    public AutoManager(
            SwerveSubsystem swerveDrive,
            FollowPath.Builder pathBuilder,
            FuelSubsystem fuelSubsystem,
            @Nullable ClimberSubsystem climberSubsystem) {
        this.swerveDrive = swerveDrive;
        this.fuelSubsystem = fuelSubsystem;
        this.pathBuilder = pathBuilder;
        this.climberSubsystem = climberSubsystem;

        final var translationLogMap = new HashMap<String, LoggedNetworkStructArray<Translation2d>>();
        final var poseLogMap = new HashMap<String, LoggedNetworkStruct<Pose2d>>();

        FollowPath.setDoubleLoggingConsumer(pair -> SmartDashboard.putNumber(pair.getFirst(), pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer(pair -> SmartDashboard.putBoolean(pair.getFirst(), pair.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(pair -> {
            final var logger = translationLogMap.computeIfAbsent(
                    "/SmartDashboard/" + pair.getFirst(),
                    key -> new LoggedNetworkStructArray<>(key, Translation2d.struct, new Translation2d[0]));
            logger.set(pair.getSecond());
        });
        FollowPath.setPoseLoggingConsumer(pair -> {
            final var logger = poseLogMap.computeIfAbsent(
                    "/SmartDashboard/" + pair.getFirst(),
                    key -> new LoggedNetworkStruct<>(key, Pose2d.struct, new Pose2d()));
            logger.set(pair.getSecond());
        });

        Field reflectionPathFieldAttempt;
        try {
            reflectionPathFieldAttempt = FollowPath.class.getDeclaredField("path");
            reflectionPathFieldAttempt.setAccessible(true);
        } catch (NoSuchFieldException e) {
            DriverStation.reportError("Could not find field 'path' in FollowPath", e.getStackTrace());
            reflectionPathFieldAttempt = null;
        }
        pathField = reflectionPathFieldAttempt;
    }

    public void autonomousInit() {
        pathBuilder.withDefaultShouldFlip();
        pathBuilder.withPoseReset(swerveDrive::resetPose);
    }

    public void teleopInit() {
        pathBuilder.withShouldFlip(() -> false);
        pathBuilder.withPoseReset(pose -> {});
    }

    public Command getPathCommandWithLeadIn(
            Path path,
            boolean stopAfter,
            boolean isFromGUI,
            @Nullable AutoStartLocation resetToLoc,
            @Nullable LinearVelocity leadInMaxVelocity) {
        if (leadInMaxVelocity == null) return getPathCommand(path, stopAfter, isFromGUI, resetToLoc);
        return Commands.sequence(
                autoLeadIn(leadInMaxVelocity, path.getStartPose(), resetToLoc),
                getPathCommand(path, stopAfter, isFromGUI, null));
    }

    public Command getPathCommand(
            Path path, boolean stopAfter, boolean isFromGUI, @Nullable AutoStartLocation resetToLoc) {
        if (isFromGUI) {
            pathBuilder.withDefaultShouldFlip();
        } else {
            pathBuilder.withShouldFlip(() -> false);
        }
        pathBuilder.withPoseReset(unused -> {});
        final var builtPath = pathBuilder.build(path.copy());
        Path newPath = null;
        try {
            newPath = (Path) (pathField != null ? pathField.get(builtPath) : null);
        } catch (ClassCastException | IllegalAccessException e) {
            DriverStation.reportError("Could not get Path from field", e.getStackTrace());
        }
        if (newPath == null) newPath = path.copy();
        final var pathPair = Pair.of(builtPath, newPath);
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            System.out.println("Started path");
                            currentPath = pathPair;
                            if (resetToLoc != null) {
                                swerveDrive.resetPose(resetToLoc.getFlippedPose());
                            }
                            pathStart = swerveDrive.getRobotPose();
                        }),
                        builtPath,
                        stopAfter ? swerveDrive.stopDrivingCommand() : Commands.none())
                .finallyDo(() -> {
                    System.out.println("Finished path");
                    if (currentPath == pathPair) {
                        currentPath = null;
                        pathStart = null;
                    }
                });
    }

    public Command getAutoCommand(AutoOptions autoOptions) {
        System.out.println("USING AUTO: " + autoOptions);
        System.out.println("USING AUTO: " + autoOptions);

        var autoCommand = Commands.none();
        var startAutoLoc = autoOptions.startLocation();
        var isLeftSide = autoOptions.startLocation().isLeftSide();
        if (autoOptions.shootAtStart1()) {
            autoCommand = shootAutoCommand(autoOptions.startLocation(), Seconds.of(4), true);
            startAutoLoc = null;
        }
        if (autoOptions.collectFromMid2()) {
            autoCommand = autoCommand.andThen(collectFromMidAndShoot(isLeftSide, startAutoLoc));
            startAutoLoc = null;
        }
        if (autoOptions.collectFromMid3()) {
            autoCommand = autoCommand.andThen(collectFromMidAndShoot(isLeftSide, startAutoLoc));
            startAutoLoc = null;
        }
        if (autoOptions.depot4()) {
            autoCommand = autoCommand.andThen(depotIntakeAndShootAutoCommand(startAutoLoc));
            startAutoLoc = null;
        }
        if (autoOptions.outpost5()) {
            autoCommand = autoCommand.andThen(outpostAndShootAutoCommand(startAutoLoc));
            startAutoLoc = null;
        }
        if (autoOptions.climb6() && climberSubsystem != null) {
            autoCommand = climberSubsystem
                    .armCommand(() -> true, () -> true)
                    .withDeadline(autoCommand.andThen(climbAutoCommand(startAutoLoc)));
            startAutoLoc = null;
        }
        return autoCommand;
    }

    private Command autoLeadIn(LinearVelocity maxVelocity, Pose2d leadInTo, @Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> {
                    final var currentPose = swerveDrive.getRobotPose();
                    final var leadInDistance = Meters.of(0);
                    final var leadInMeters = leadInDistance.in(Meters);
                    final var currentDiff = leadInTo.getTranslation().minus(currentPose.getTranslation());
                    if (MathUtil.isNear(0.0, currentDiff.getNorm(), 0.1)) {
                        return Commands.none();
                    }
                    final var clampedDiff = new Translation2d(
                            MathUtil.clamp(currentDiff.getX(), -leadInMeters, leadInMeters),
                            MathUtil.clamp(currentDiff.getY(), -leadInMeters, leadInMeters));
                    final var newPose = new Pose2d(
                            leadInTo.getX() - clampedDiff.getX(),
                            leadInTo.getY() - clampedDiff.getY(),
                            leadInTo.getRotation());
                    final var path = new Path(
                            new Path.PathConstraints()
                                    .setMaxVelocityMetersPerSec(
                                            new Path.RangedConstraint(maxVelocity.in(MetersPerSecond), 1, 2))
                                    .setEndTranslationToleranceMeters(0.4),
                            new Path.Waypoint(newPose, 0.3),
                            new Path.Waypoint(leadInTo, 0.4));
                    return getPathCommand(path, false, false, resetToLoc);
                },
                Set.of(swerveDrive));
    }

    private Command climbAutoCommand(@Nullable AutoStartLocation resetToLoc) {
        if (climberSubsystem == null) return Commands.none();
        return climberSubsystem
                .armCommand(() -> true, () -> true)
                .withDeadline(getPathCommand(alignClimbPath, false, true, resetToLoc)
                        .andThen(swerveDrive
                                .driveFieldOrientedCommand(
                                        () -> MetersPerSecond.of(0.2 * (shouldFlip() ? -1 : 1)),
                                        MetersPerSecond::zero,
                                        DegreesPerSecond::zero)
                                .withTimeout(Seconds.of(1.0))))
                .andThen(climberSubsystem.climbCommand(() -> true, () -> false));
    }

    private Command outpostAndShootAutoCommand(@Nullable AutoStartLocation resetToLoc) {
        return fuelSubsystem
                .idleCommand()
                .withDeadline(
                        getPathCommand(toOutpostPath, true, true, resetToLoc).andThen(Commands.waitTime(Seconds.of(3))))
                .andThen(pathInFrontOfHubAndShoot(null));
    }

    private Command depotIntakeAndShootAutoCommand(@Nullable AutoStartLocation resetToLoc) {
        return fuelSubsystem
                .intakeCommand()
                .withDeadline(getPathCommandWithLeadIn(depotPath, false, true, resetToLoc, MetersPerSecond.of(1.6)))
                .andThen(shootAutoCommand(AutoStartLocation.LEFT_INNER_BUMP, Seconds.of(4), false));
    }

    private Command pathInFrontOfHubAndShoot(@Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> {
                    final var pose = new Pose2d(
                            FieldConstants.HUB_BLUE.getMeasureX().minus(distanceSupplier.get()),
                            FieldConstants.HUB_BLUE.getMeasureY(),
                            Rotation2d.k180deg);
                    final var path = new Path(new Path.Waypoint(pose));
                    path.setPathConstraints(new Path.PathConstraints().setMaxVelocityMetersPerSec(1.2));
                    return fuelSubsystem
                            .windUpCommand()
                            .withDeadline(getPathCommand(path, true, false, resetToLoc))
                            .andThen(fuelSubsystem.launchCommand(true));
                },
                Set.of(swerveDrive, fuelSubsystem));
    }

    private Command shootFromStartAutoWIP(Distance yPos, @Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> {
                    final var startPose = new Pose2d(
                            FieldConstants.HUB_BLUE
                                    .getMeasureX()
                                    .minus(FieldConstants.HUB_WIDTH_X.div(2))
                                    .minus(PhysicalConstants.ROBOT_LENGTH_X.div(2)), // 145.985in/3.708m
                            yPos,
                            Rotation2d.k180deg);
                    final var distance = distanceSupplier.get().in(Meters);
                    final var distanceY =
                            yPos.minus(FieldConstants.HUB_BLUE.getMeasureY()).in(Meters);
                    if (Math.abs(distanceY) > Math.abs(distance)) {
                        DriverStation.reportError("Distance Y cannot be greater than distance", true);
                        return Commands.none();
                    }
                    final var distanceX = Math.sqrt(Math.pow(distance, 2) - Math.pow(distanceY, 2));
                    final var angle = 180.0 - Math.toDegrees(Math.atan2(distanceY, distanceX));
                    final var endPose = new Pose2d(
                            FieldConstants.HUB_BLUE.getMeasureX().minus(Meters.of(distanceX)),
                            yPos,
                            Rotation2d.fromDegrees(angle));
                    final var path = new Path(new Path.Waypoint(startPose), new Path.Waypoint(endPose));
                    path.setPathConstraints(new Path.PathConstraints().setMaxVelocityMetersPerSec(0.4));
                    return fuelSubsystem
                            .windUpCommand()
                            .withDeadline(getPathCommand(path, true, false, resetToLoc))
                            .andThen(fuelSubsystem.launchCommand(true));
                },
                Set.of(swerveDrive, fuelSubsystem));
    }

    public Command moveFromHubAndShoot(@Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> {
                    final var dashboardDistance = distanceSupplier.get();
                    var newPose = new Pose2d(
                            new Translation2d(
                                    FieldConstants.HUB_BLUE.getMeasureX().minus(dashboardDistance),
                                    FieldConstants.HUB_BLUE.getMeasureY()),
                            Rotation2d.k180deg);
                    if (shouldFlip()) {
                        newPose = FlippingUtil.flipFieldPose(newPose);
                    }
                    final var path = new Path(new Path.Waypoint(newPose));
                    path.setPathConstraints(new Path.PathConstraints()
                            .setMaxVelocityMetersPerSec(1.5)
                            .setMaxAccelerationMetersPerSec2(4));
                    return fuelSubsystem
                            .windUpCommand()
                            .withDeadline(getPathCommand(path, true, false, resetToLoc))
                            .andThen(fuelSubsystem.launchCommand(true));
                },
                Set.of(swerveDrive, fuelSubsystem));
    }

    public Command collectFromMidAndShoot(boolean leftSide, @Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> Commands.sequence(
                        fuelSubsystem
                                .intakeCommand()
                                .withDeadline(Commands.sequence(
                                        goOverBump(leftSide, true, false, resetToLoc),
                                        getPathCommand(
                                                leftSide ? collectMidFromLeftPath : collectMidFromRightPath,
                                                false,
                                                true,
                                                null),
                                        goOverBump(leftSide, false, false, null))),
                        shootAutoCommand(
                                leftSide ? AutoStartLocation.LEFT_INNER_BUMP : AutoStartLocation.RIGHT_INNER_BUMP,
                                Seconds.of(15),
                                false)),
                Set.of(swerveDrive, fuelSubsystem));
    }

    public Command shootAutoCommand(AutoStartLocation location, Time launchDuration, boolean isStart) {
        return Commands.sequence(
                fuelSubsystem
                        .windUpCommand()
                        .withDeadline(Commands.defer(
                                () -> {
                                    final Path originalPath =
                                            switch (location) {
                                                case LEFT_INNER_BUMP -> startLeftInnerBumpShootPath;
                                                case LEFT_HUB -> startLeftHubShootPath;
                                                case RIGHT_INNER_BUMP -> startRightInnerBumpShootPath;
                                            };
                                    final Pair<Path.PathElement, Path.PathElementConstraint> lastPathWithConstraint =
                                            getLastPathWithConstraint(originalPath);
                                    final var path = new Path(
                                            lastPathWithConstraint.getFirst().copy());
                                    final var constraints =
                                            (Path.WaypointConstraint) lastPathWithConstraint.getSecond();
                                    path.setPathConstraints(copyConstraintsFrom(
                                                    new Path.PathConstraints(), constraints, 0, 0)
                                            .setEndTranslationToleranceMeters(
                                                    originalPath.getEndTranslationToleranceMeters())
                                            .setEndRotationToleranceDeg(originalPath.getEndRotationToleranceDeg()));
                                    return getPathCommand(path, true, true, isStart ? location : null);
                                },
                                Set.of(swerveDrive))),
                fuelSubsystem
                        .launchCommand(true)
                        .withDeadline(Commands.waitUntil(fuelSubsystem.isReadyToLaunchTrigger())
                                .andThen(Commands.waitTime(launchDuration))));
    }

    public Command goOverBump(
            boolean leftSide, boolean intoCenter, boolean stopAfter, @Nullable AutoStartLocation resetToLoc) {
        return Commands.defer(
                () -> {
                    final var isRed = DriverStation.getAlliance().orElse(null) == Alliance.Red;
                    final Distance distanceYFromHub = Inches.of(60);
                    final Distance distanceXFromHub = Meters.of(1.2);
                    final Translation2d hub = isRed ? FieldConstants.HUB_RED : FieldConstants.HUB_BLUE;
                    final boolean increasingX = !isRed == intoCenter;
                    final boolean posY = !isRed == leftSide;
                    final Distance y = hub.getMeasureY().plus(posY ? distanceYFromHub : distanceYFromHub.unaryMinus());
                    final Distance x1 =
                            hub.getMeasureX().plus(increasingX ? distanceXFromHub.unaryMinus() : distanceXFromHub);
                    final Distance x2 =
                            hub.getMeasureX().plus(increasingX ? distanceXFromHub : distanceXFromHub.unaryMinus());
                    final Rotation2d angle = Rotation2d.fromDegrees(30 + 150 + (intoCenter == !isRed ? 180 : 0));
                    final Pose2d pose1 = new Pose2d(x1, y, angle);
                    final Pose2d pose2 = new Pose2d(x2, y, angle);
                    final Path path = new Path(new Path.Waypoint(pose1, 0.5), new Path.Waypoint(pose2));
                    path.setPathConstraints(new Path.PathConstraints()
                            .setEndTranslationToleranceMeters(0.5)
                            .setEndRotationToleranceDeg(30));
                    return getPathCommand(path, stopAfter, false, resetToLoc);
                },
                Set.of(swerveDrive));
    }

    public Command moveRobotDriverOriented(
            Translation2d translation, @Nullable Rotation2d rotation, Path.@Nullable PathConstraints constraints) {
        return Commands.defer(
                () -> {
                    final Pose2d currentPose = swerveDrive.getRobotPose();
                    final var flipSign = shouldFlip() ? -1 : 1;
                    final Pose2d newPose = new Pose2d(
                            currentPose.getTranslation().plus(translation.times(flipSign)),
                            rotation == null ? currentPose.getRotation() : rotation.times(flipSign));
                    final Path path = new Path(new Path.Waypoint(currentPose), new Path.Waypoint(newPose));
                    if (constraints != null) {
                        path.setPathConstraints(constraints);
                    }
                    return getPathCommand(path, true, false, null);
                },
                Set.of(swerveDrive));
    }

    public Command testAuto() {
        return Commands.defer(
                () -> {
                    Pose2d currentPose = swerveDrive.getRobotPose();
                    final var xDist = 1;
                    final var yDist = 1;
                    final var transforms = List.of(
                            new Transform2d(xDist, 0, Rotation2d.kZero),
                            new Transform2d(0, yDist, Rotation2d.kZero),
                            new Transform2d(-xDist, -yDist, Rotation2d.kZero));
                    final var waypoints = new ArrayList<Path.PathElement>();
                    // waypoints.add(new Path.Waypoint(currentPose));
                    var rotation = 0;
                    final var startRotation = currentPose.getRotation();
                    for (int i = 0; i < transforms.size(); i++) {
                        final var transform = transforms.get(i);
                        // rotation += 90;
                        currentPose = new Pose2d(
                                currentPose.getMeasureX().plus(transform.getMeasureX()),
                                currentPose.getMeasureY().plus(transform.getMeasureY()),
                                i == transforms.size() - 1 ? startRotation : Rotation2d.fromDegrees(rotation));
                        final var waypoint = new Path.Waypoint(currentPose, 0.15);
                        waypoints.add(waypoint);
                    }
                    final Path path = new Path(waypoints);
                    return getPathCommand(path, true, false, null);
                },
                Set.of(swerveDrive));
    }

    public Command testOnePointPath() {
        return Commands.defer(
                () -> {
                    final var pose = swerveDrive.getRobotPose();
                    final Path path = new Path(new Path.Waypoint(new Pose2d(
                            pose.getMeasureX().plus(testAutoX.get()),
                            pose.getMeasureY().plus(testAutoY.get()),
                            pose.getRotation()
                                    .plus(Rotation2d.fromDegrees(
                                            testAutoAngle.get().in(Degrees))))));
                    path.setPathConstraints(new Path.PathConstraints().setEndTranslationToleranceMeters(0.04));
                    return getPathCommandWithLeadIn(path, true, false, null, testAutoLeadInVelocity.get());
                },
                Set.of(swerveDrive));
    }

    private boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(null) == Alliance.Red;
    }

    private Pair<Path.PathElement, Path.PathElementConstraint> getLastPathWithConstraint(Path path) {
        return path.getPathElementsWithConstraints()
                .get(path.getPathElementsWithConstraints().size() - 1);
    }

    public @Nullable List<Pose2d> getCurrentPoses() {
        if (currentPath == null) {
            return null;
        }
        final Pose2d currentPose = swerveDrive.getRobotPose();
        final Path path = currentPath.getSecond();
        final var states = new ArrayList<Pose2d>();
        if (pathStart != null) states.add(pathStart);
        Rotation2d rotation = currentPose.getRotation();
        for (var element : path.getPathElements()) {
            if (element instanceof Path.Waypoint waypoint) {
                states.add(new Pose2d(
                        waypoint.translationTarget().translation(),
                        waypoint.rotationTarget().rotation()));
                rotation = waypoint.rotationTarget().rotation();
            } else if (element instanceof Path.TranslationTarget translationTarget) {
                states.add(new Pose2d(translationTarget.translation(), rotation));
            } else if (element instanceof Path.RotationTarget rotationTarget) {
                rotation = rotationTarget.rotation();
            }
        }
        return states;
    }

    /*public @Nullable List<Pose2d> getCompletedPoses() {
        final var poses = getCurrentPoses();
        if (poses == null || currentPath == null) return null;
        final var stage = currentPath.getFirst().getCurrentTranslationElementIndex();
        return poses.subList(0, Math.min(stage + 1, poses.size()));
    }

    public @Nullable List<Pose2d> getPosesToComplete() {
        final var poses = getCurrentPoses();
        if (poses == null || currentPath == null) return null;
        final var stage = currentPath.getFirst().getCurrentTranslationElementIndex();
        return poses.subList(Math.min(stage + 1, poses.size()), poses.size());
    }*/

    /**
     * Calculates the clamped projection ratio of a point onto a segment.
     *
     * @param segmentStart The start of the segment
     * @param segmentEnd The end of the segment
     * @param point The point to project
     * @return Projection ratio along the segment in [0, 1]
     */
    private double calculateSegmentProjectionT(
            Translation2d segmentStart, Translation2d segmentEnd, Translation2d point) {
        double dx = segmentEnd.getX() - segmentStart.getX();
        double dy = segmentEnd.getY() - segmentStart.getY();
        double segmentLengthSquared = dx * dx + dy * dy;
        if (segmentLengthSquared < 1e-6) {
            return 0.0;
        }

        double dxPoint = point.getX() - segmentStart.getX();
        double dyPoint = point.getY() - segmentStart.getY();
        double t = (dxPoint * dx + dyPoint * dy) / segmentLengthSquared;
        return Math.max(0.0, Math.min(1.0, t));
    }

    private Path.PathConstraints copyConstraintsFrom(
            Path.PathConstraints constraintsToApplyTo,
            Path.PathElementConstraint constraintsToCopyFrom,
            int startOrdinal,
            int endOrdinal) {
        if (constraintsToCopyFrom instanceof Path.WaypointConstraint waypointConstraint) {
            return constraintsToApplyTo
                    .setMaxVelocityMetersPerSec(new Path.RangedConstraint(
                            waypointConstraint.maxVelocityMetersPerSec(), startOrdinal, endOrdinal))
                    .setMaxVelocityDegPerSec(new Path.RangedConstraint(
                            waypointConstraint.maxVelocityDegPerSec(), startOrdinal, endOrdinal))
                    .setMaxAccelerationMetersPerSec2(new Path.RangedConstraint(
                            waypointConstraint.maxAccelerationMetersPerSec2(), startOrdinal, endOrdinal))
                    .setMaxAccelerationDegPerSec2(new Path.RangedConstraint(
                            waypointConstraint.maxAccelerationDegPerSec2(), startOrdinal, endOrdinal));
        } else if (constraintsToCopyFrom instanceof Path.TranslationTargetConstraint translationTargetConstraint) {
            return constraintsToApplyTo
                    .setMaxVelocityMetersPerSec(new Path.RangedConstraint(
                            translationTargetConstraint.maxVelocityMetersPerSec(), startOrdinal, endOrdinal))
                    .setMaxAccelerationMetersPerSec2(new Path.RangedConstraint(
                            translationTargetConstraint.maxAccelerationMetersPerSec2(), startOrdinal, endOrdinal));
        } else if (constraintsToCopyFrom instanceof Path.RotationTargetConstraint rotationTargetConstraint) {
            return constraintsToApplyTo
                    .setMaxVelocityDegPerSec(new Path.RangedConstraint(
                            rotationTargetConstraint.maxVelocityDegPerSec(), startOrdinal, endOrdinal))
                    .setMaxAccelerationDegPerSec2(new Path.RangedConstraint(
                            rotationTargetConstraint.maxAccelerationDegPerSec2(), startOrdinal, endOrdinal));
        } else {
            throw new IllegalArgumentException("Unknown Path.PathElementConstraint type: "
                    + constraintsToCopyFrom.getClass().getName());
        }
    }

    public enum AutoStartLocation {
        LEFT_INNER_BUMP(new Pose2d(Meters.of(3.563), Meters.of(5.111), Rotation2d.k180deg)),
        LEFT_HUB(new Pose2d(Meters.of(3.708), Meters.of(4.157), Rotation2d.k180deg)),
        RIGHT_INNER_BUMP(new Pose2d(Meters.of(3.563), Meters.of(2.958), Rotation2d.k180deg)),
        ;

        private final Pose2d pose;
        private final boolean isLeftSide;

        AutoStartLocation(Pose2d pose) {
            this.pose = pose;
            isLeftSide = pose.getMeasureY().gt(FieldConstants.FIELD_WIDTH_Y.div(2));
        }

        public Pose2d getPose() {
            return pose;
        }

        public Pose2d getFlippedPose() {
            if (DriverStation.getAlliance().orElse(null) == Alliance.Red) {
                return FlippingUtil.flipFieldPose(pose);
            }
            return pose;
        }

        public boolean isLeftSide() {
            return isLeftSide;
        }
    }

    public record AutoOptions(
            AutoStartLocation startLocation,
            boolean shootAtStart1,
            boolean collectFromMid2,
            boolean collectFromMid3,
            boolean depot4,
            boolean outpost5,
            boolean climb6) {}
}
