package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.BuildConstants;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.urcl.URCL;

@NullMarked
public class Robot extends TimedRobot {
    private @Nullable Command autonomousCommand;
    private @Nullable RobotContainer robotContainer;

    /** This function is run when the robot is first started up and should be used for any initialization code. */
    @Override
    public void robotInit() {
        if (isReal()) {
            startLogging();
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.

        robotContainer = new RobotContainer();
    }

    @SuppressWarnings("DataFlowIssue")
    private void startLogging() {
        DataLogManager.start();
        DataLogManager.logConsoleOutput(true);
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        URCL.start(Map.ofEntries(
                Map.entry(1, "Front-Left Drive"),
                Map.entry(2, "Front-Left Angle"),
                Map.entry(3, "Front-Right Drive"),
                Map.entry(4, "Front-Right Angle"),
                Map.entry(5, "Back-Right Drive"),
                Map.entry(6, "Back-Right Angle"),
                Map.entry(7, "Back-Left Drive"),
                Map.entry(8, "Back-Left Angle"),
                Map.entry(9, "Climber"),
                Map.entry(10, "Intake-Launcher Left"),
                Map.entry(11, "Intake-Launcher Right"),
                Map.entry(12, "Indexer"),
                Map.entry(13, "Pigeon2"),
                Map.entry(14, "Power Distribution Hub"),
                Map.entry(16, "CANdle")));

        final var metadataTable = NetworkTableInstance.getDefault().getTable("Metadata");
        metadataTable.getEntry("BuildDate").setString(BuildConstants.BUILD_DATE);
        metadataTable.getEntry("GitBranch").setString(BuildConstants.GIT_BRANCH);
        metadataTable.getEntry("GitDate").setString(BuildConstants.GIT_DATE);
        metadataTable
                .getEntry("GitDirty")
                .setString(
                        switch (BuildConstants.DIRTY) {
                            case 0 -> "All changes committed";
                            case 1 -> "Uncommitted changes";
                            default -> "Unknown";
                        });
        metadataTable.getEntry("GitSHA").setString(BuildConstants.GIT_SHA);
        metadataTable.getEntry("GitRevision").setString(String.valueOf(BuildConstants.GIT_REVISION));
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing finished or
        // interrupted commands, and running subsystem periodic() methods. This must be called from the
        // robot's periodic block in order for anything in the Command-based framework to work.
        if (robotContainer != null) robotContainer.preSchedulerUpdate();
        CommandScheduler.getInstance().run();
        if (robotContainer != null) robotContainer.postSchedulerUpdate();

        getRobotContainer().periodic();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        getRobotContainer().autonomousInit();
        autonomousCommand = getRobotContainer().getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        if (robotContainer != null) robotContainer.teleopInit();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Add any periodic code for teleop mode here
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        if (robotContainer != null) {
            robotContainer.simulationInit();
        }
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        if (robotContainer != null) {
            robotContainer.simulationPeriodic();
        }
    }

    @Override
    public void close() {
        super.close();
    }

    private RobotContainer getRobotContainer() {
        if (Objects.isNull(robotContainer)) {
            throw new IllegalStateException("RobotContainer is not yet initialized");
        }
        return robotContainer;
    }
}
