package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.enums.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class AutoManager {
    private final SwerveSubsystem swerveDrive;
    private final FollowPath.Builder pathBuilder;
    private final FuelSubsystem fuelSubsystem;

    // Built-in paths
    private final Path shootFromStartPath = new Path("shoot_from_start");
    private final Supplier<Distance> distanceSupplier =
            new LoggedNetworkUnit<>("Auto/Move from hub & shoot distance (m)", Meters.of(-1.0));

    public AutoManager(SwerveSubsystem swerveDrive, FollowPath.Builder pathBuilder, FuelSubsystem fuelSubsystem) {
        this.swerveDrive = swerveDrive;
        this.fuelSubsystem = fuelSubsystem;
        this.pathBuilder = pathBuilder;
    }

    public FollowPath getPathCommand(Path path) {
        return pathBuilder.build(path);
    }

    private Path getAutoPath() {
        return shootFromStartPath; // selectable via dashboard?
    }

    public Command getAutoCommand() {
        return shootFromStartAutoCommand(); // selectable via dashboard?
    }

    public void orientAutoModuleOrientations() {
        swerveDrive.orientModuleOrientationsForPath(getAutoPath());
    }

    private Command shootFromStartAutoCommand() {
        return Commands.sequence(
                getPathCommand(shootFromStartPath),
                fuelSubsystem.windUpAndLaunchCommand().withTimeout(Seconds.of(15)));
    }

    /*public Command moveFromHubAndShoot() {
        return Commands.defer(() -> {
            final var dashboardDistance = distanceSupplier.get();
            final var currentRot = swerveDrive.getRobotPose().getRotation().getDegrees();
            final var invertXY = MathUtil.isNear(90.0, Math.abs(currentRot) % 180, 45.0);
            var xPos = (invertXY ? PhysicalConstants.ROBOT_WIDTH_Y : PhysicalConstants.ROBOT_LENGTH_X).div(2);
            final var distanceWithOffset = dashboardDistance.plus(FieldConstants.HUB_WIDTH_X.div(2)).plus(
                PhysicalConstants.ROBOT_LENGTH_X.div(2);
            )
            moveRobotDriverOriented(new Transform2d())
        }, Set.of(swerveDrive, fuelSubsystem));
    }*/

    public Command moveRobotDriverOriented(Transform2d transform) {
        return Commands.defer(
                () -> {
                    final Pose2d currentPose = swerveDrive.getRobotPose();
                    final Pose2d newPose = currentPose.plus(transform);
                    final Path path = new Path(new Path.Waypoint(newPose));
                    return buildPathWithSpecifiedFlip(path, false);
                },
                Set.of(swerveDrive));
    }

    public Command testAuto() {
        return Commands.defer(
                () -> {
                    Path.setDefaultGlobalConstraints(Constants.BLineConstants.GLOBAL_CONSTRAINTS);
                    Pose2d currentPose = swerveDrive.getRobotPose();
                    final var transforms = List.of(
                            new Transform2d(1, 0, Rotation2d.kZero),
                            new Transform2d(0, 1, Rotation2d.kZero),
                            new Transform2d(-1, -1, Rotation2d.kZero));
                    final var waypoints = new ArrayList<Path.PathElement>();
                    waypoints.add(new Path.Waypoint(currentPose));
                    var rotation = 0;
                    for (final var transform : transforms) {
                        rotation += 90;
                        currentPose = new Pose2d(
                                currentPose.getMeasureX().plus(transform.getMeasureX()),
                                currentPose.getMeasureY().plus(transform.getMeasureY()),
                                Rotation2d.fromDegrees(rotation));
                        waypoints.add(new Path.Waypoint(currentPose));
                    }
                    final Path path = new Path(waypoints);
                    path.setPathConstraints(new Path.PathConstraints().setEndTranslationToleranceMeters(0.02));
                    return buildPathWithSpecifiedFlip(path, false);
                },
                Set.of(swerveDrive));
    }

    private FollowPath buildPathWithSpecifiedFlip(Path path, boolean flip) {
        pathBuilder.withShouldFlip(() -> flip);
        final var builtPath = pathBuilder.build(path);
        pathBuilder.withDefaultShouldFlip();
        return builtPath;
    }
}
