package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.enums.Constants.FieldConstants;
import frc.robot.util.enums.Constants.PhysicalConstants;
import java.util.ArrayList;
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
    private final ClimberSubsystem climberSubsystem;

    // Built-in paths
    private final Path startLeftHubShootPath = new Path("start_left_hub_shoot");
    private final Path startLeftInnerBumpShootPath = new Path("start_left_inner_bump_shoot");
    private final Path startRightInnerBumpShootPath = new Path("start_right_inner_bump_shoot");
    private final Path alignClimbPath = new Path("align_climb");
    private final Supplier<Distance> distanceSupplier =
            new LoggedNetworkUnit<>("Auto/Move from hub & shoot distance (m)", Meters.of(1.8));

    public AutoManager(
            SwerveSubsystem swerveDrive,
            FollowPath.Builder pathBuilder,
            FuelSubsystem fuelSubsystem,
            ClimberSubsystem climberSubsystem) {
        this.swerveDrive = swerveDrive;
        this.fuelSubsystem = fuelSubsystem;
        this.pathBuilder = pathBuilder;
        this.climberSubsystem = climberSubsystem;
    }

    public void teleopInit() {
        pathBuilder.withShouldFlip(() -> false);
        pathBuilder.withPoseReset(pose -> {});
    }

    public FollowPath getPathCommand(Path path) {
        return pathBuilder.build(path);
    }

    public Command getAutoCommand(AutoOptions autoOptions) {
        System.out.println("USING AUTO: " + autoOptions);
        System.out.println("USING AUTO: " + autoOptions);

        var autoCommand = autoOptions.climb ? climberSubsystem.armCommand(() -> true) : Commands.none();
        autoCommand = autoCommand.withDeadline(shootFromStartAutoCommand(autoOptions.startLocation));
        if (autoOptions.climb) {
            autoCommand = autoCommand.andThen(climbAutoCommand());
        }
        return autoCommand;
    }

    private Command climbAutoCommand() {
        return climberSubsystem
                .armCommand(() -> true)
                .withDeadline(getPathCommand(alignClimbPath)
                        .andThen(swerveDrive
                                .driveFieldOrientedCommand(
                                        () -> MetersPerSecond.of(0.2 * (shouldFlip() ? -1 : 1)),
                                        MetersPerSecond::zero,
                                        DegreesPerSecond::zero)
                                .withTimeout(Seconds.of(1.0))))
                .andThen(climberSubsystem.climbCommand(() -> true));
    }

    private Command shootFromStartAutoCommand(AutoStartLocation startLocation) {
        return Commands.sequence(
                fuelSubsystem
                        .windUpCommand()
                        .withDeadline(getPathCommand(
                                switch (startLocation) {
                                    case LEFT_INNER_BUMP -> startLeftInnerBumpShootPath;
                                    case LEFT_HUB -> startLeftHubShootPath;
                                    case RIGHT_INNER_BUMP -> startRightInnerBumpShootPath;
                                })),
                fuelSubsystem.windUpAndLaunchCommand().withTimeout(Seconds.of(10)));
    }

    private Command shootFromStartAutoWIP(Distance yPos) {
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
                            .withDeadline(pathBuilder.build(path))
                            .andThen(fuelSubsystem.windUpAndLaunchCommand());
                },
                Set.of(swerveDrive, fuelSubsystem));
    }

    public Command moveFromHubAndShoot() {
        return Commands.defer(
                () -> {
                    final var dashboardDistance = distanceSupplier.get();
                    final var currentPose = swerveDrive.getRobotPose();
                    var newPose = new Pose2d(
                            new Translation2d(
                                    FieldConstants.HUB_BLUE.getMeasureX().minus(dashboardDistance),
                                    FieldConstants.HUB_BLUE.getMeasureY()),
                            Rotation2d.k180deg);
                    if (shouldFlip()) {
                        newPose = FlippingUtil.flipFieldPose(newPose);
                    }
                    final var path = new Path(/*new Path.Waypoint(currentPose), */ new Path.Waypoint(newPose));
                    path.setPathConstraints(new Path.PathConstraints()
                            .setMaxVelocityMetersPerSec(1.5)
                            .setMaxAccelerationMetersPerSec2(4));
                    return fuelSubsystem
                            .windUpCommand()
                            .withDeadline(pathBuilder.build(path))
                            .andThen(fuelSubsystem.windUpAndLaunchCommand());
                },
                Set.of(swerveDrive, fuelSubsystem));
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
                    return pathBuilder.build(path);
                },
                Set.of(swerveDrive));
    }

    public Command testAuto() {
        return Commands.defer(
                () -> {
                    Pose2d currentPose = swerveDrive.getRobotPose();
                    final var transforms = List.of(
                            new Transform2d(1, 0, Rotation2d.kZero),
                            new Transform2d(0, 1, Rotation2d.kZero),
                            new Transform2d(-1, -1, Rotation2d.kZero));
                    final var waypoints = new ArrayList<Path.PathElement>();
                    // waypoints.add(new Path.Waypoint(currentPose));
                    var rotation = 0;
                    for (final var transform : transforms) {
                        rotation += 90;
                        currentPose = new Pose2d(
                                currentPose.getMeasureX().plus(transform.getMeasureX()),
                                currentPose.getMeasureY().plus(transform.getMeasureY()),
                                Rotation2d.fromDegrees(rotation));
                        final var waypoint = new Path.Waypoint(currentPose, 0.15);
                        waypoints.add(waypoint);
                    }
                    final Path path = new Path(waypoints);
                    path.setPathConstraints(new Path.PathConstraints()
                            .setMaxVelocityMetersPerSec(1.5)
                            .setMaxAccelerationMetersPerSec2(1.5)
                            .setEndTranslationToleranceMeters(0.02));
                    return pathBuilder.build(path);
                },
                Set.of(swerveDrive));
    }

    private boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(null) == Alliance.Red;
    }

    public enum AutoStartLocation {
        LEFT_INNER_BUMP,
        LEFT_HUB,
        RIGHT_INNER_BUMP,
    }

    public record AutoOptions(AutoStartLocation startLocation, boolean climb, boolean outpost) {}
}
