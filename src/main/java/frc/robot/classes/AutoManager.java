package frc.robot.classes;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class AutoManager {
    private final SwerveSubsystem swerveDrive;
    private final FollowPath.Builder pathBuilder;
    private final FuelSubsystem fuelSubsystem;

    // Built-in paths
    private final Path shootFromStartPath = new Path("shoot_from_start");

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
}
