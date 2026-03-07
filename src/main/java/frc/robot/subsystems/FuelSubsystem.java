// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.FuelConstants;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@NullMarked
public class FuelSubsystem extends SubsystemBase {

    private FuelAction currentState;
    private final ShooterCalculator shooterCalculator;

    private final Trigger launchingTrigger;
    private final Trigger ejectingTrigger;
    private final Trigger intakingTrigger;
    private final Trigger windingUpTrigger;

    private final SmartMotorController intakeLauncherController;
    private final SmartMotorController indexerController;

    private final FlyWheel intakeLauncher;
    private final FlyWheel indexer;

    private boolean useCustomVelocity = true;
    private DashboardFuelAction dashboardFuelAction = DashboardFuelAction.IDLE;

    private final Supplier<AngularVelocity> ejectVelocityIntakeLauncher = new LoggedNetworkUnit<>(
            "Fuel/Eject Velocity Intake-Launcher", FuelConstants.EJECT_VELOCITY_INTAKE_LAUNCHER);
    private final Supplier<AngularVelocity> ejectVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Eject Velocity Indexer", FuelConstants.EJECT_VELOCITY_INDEXER);
    private final Supplier<AngularVelocity> unJamVelocityIntakeLauncher = new LoggedNetworkUnit<>(
            "Fuel/Unjam Velocity Intake-Launcher", FuelConstants.UNJAM_VELOCITY_INTAKE_LAUNCHER);
    private final Supplier<AngularVelocity> unJamVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Unjam Velocity Indexer", FuelConstants.UNJAM_VELOCITY_INDEXER);
    private final Supplier<AngularVelocity> intakeVelocityIntakeLauncher = new LoggedNetworkUnit<>(
            "Fuel/Intake Velocity Intake-Launcher", FuelConstants.INTAKE_VELOCITY_INTAKE_LAUNCHER);
    private final Supplier<AngularVelocity> intakeVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Intake Velocity Indexer", FuelConstants.INTAKE_VELOCITY_INDEXER);
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> launchVelocityIntakeLauncher =
            new LoggedNetworkUnit<>("Fuel/Launch Velocity Intake-Launcher", RotationsPerSecond.of(47.1));
    private final Supplier<AngularVelocity> launchVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Launch Velocity Indexer", FuelConstants.LAUNCH_VELOCITY_INDEXER);
    private final Supplier<AngularVelocity> windUpVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Windup Velocity Indexer", FuelConstants.WINDUP_VELOCITY_INDEXER);

    public FuelSubsystem(ShooterCalculator shooterCalculator, MultiMotorInfoSendable motorInfo) {
        this.shooterCalculator = shooterCalculator;

        final var intakeLauncherLeftSMCConfig = new SmartMotorControllerConfig(this)
                .withGearing(FuelConstants.INTAKE_LAUNCHER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INTAKE_LAUNCHER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INTAKE_LAUNCHER_INVERTED)
                .withVoltageCompensation(FuelConstants.INTAKE_LAUNCHER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INTAKE_LAUNCHER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT)
                .withFeedforward(new SimpleMotorFeedforward(0.15, 0.192))
                .withClosedLoopController(new PIDController(0.003, 0.0, 0.1))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMotorInverted(true)
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
        final var followerIntakeLauncherSMCConfig = new SmartMotorControllerConfig(this)
                .withGearing(FuelConstants.INTAKE_LAUNCHER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INTAKE_LAUNCHER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INTAKE_LAUNCHER_INVERTED)
                .withVoltageCompensation(FuelConstants.INTAKE_LAUNCHER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INTAKE_LAUNCHER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT)
                .withControlMode(ControlMode.OPEN_LOOP);
        final var indexerSMCConfig = new SmartMotorControllerConfig(this)
                .withFeedforward(new SimpleMotorFeedforward(0.03, 0.23))
                .withClosedLoopController(new PIDController(0.002, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
                .withGearing(FuelConstants.INDEXER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INDEXER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INDEXER_INVERTED)
                .withVoltageCompensation(FuelConstants.INDEXER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INDEXER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INDEXER_CURRENT_LIMIT)
                .withMotorInverted(true)
                .withFollowers();

        final var intakeLauncherLeftSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
        final var intakeLauncherRightSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        final var indexerSparkMax = new SparkMax(FuelConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        // apply config
        new SparkWrapper(intakeLauncherRightSparkMax, DCMotor.getNEO(1), followerIntakeLauncherSMCConfig);

        intakeLauncherLeftSMCConfig.withFollowers(Pair.of(intakeLauncherRightSparkMax, true));
        intakeLauncherController =
                new SparkWrapper(intakeLauncherLeftSparkMax, DCMotor.getNEO(2), intakeLauncherLeftSMCConfig);
        indexerController = new SparkWrapper(indexerSparkMax, DCMotor.getNEO(1), indexerSMCConfig);

        intakeLauncher = new FlyWheel(new FlyWheelConfig(intakeLauncherController)
                .withDiameter(FuelConstants.WHEEL_RADIUS.times(2))
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH));
        indexer = new FlyWheel(new FlyWheelConfig(indexerController)
                .withDiameter(FuelConstants.WHEEL_RADIUS.times(2))
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH));

        setDefaultCommand(Commands.select(
                Map.of(
                        DashboardFuelAction.IDLE, idleCommand(),
                        DashboardFuelAction.INTAKE, intakeCommand(),
                        DashboardFuelAction.EJECT, ejectCommand(),
                        DashboardFuelAction.LAUNCH, windUpAndLaunchCommand(),
                        DashboardFuelAction.UNJAM, unjamCommand()),
                () -> dashboardFuelAction));
        currentState = FuelAction.IDLE;
        launchingTrigger = new Trigger(() -> currentState == FuelAction.LAUNCH);
        ejectingTrigger = new Trigger(() -> currentState == FuelAction.EJECT);
        intakingTrigger = new Trigger(() -> currentState == FuelAction.INTAKE);
        windingUpTrigger = new Trigger(() -> currentState == FuelAction.WIND_UP);

        motorInfo.addMotor(intakeLauncherLeftSparkMax, "Intake-Launcher Left");
        motorInfo.addMotor(intakeLauncherRightSparkMax, "Intake-Launcher Right");
        motorInfo.addMotor(indexerSparkMax, "Indexer");

        setupSmartDashboard();
    }

    private void setupSmartDashboard() {
        SmartDashboard.putData(
                "Intake-Launcher",
                (builder) -> builder.addDoubleProperty(
                        "Velocity", () -> intakeLauncher.getSpeed().in(RotationsPerSecond), null));
        SmartDashboard.putData(
                "Indexer",
                (builder) -> builder.addDoubleProperty(
                        "Velocity", () -> indexer.getSpeed().in(RotationsPerSecond), null));
        SmartDashboard.putData(
                "Intake-Launcher PID",
                new PIDSendable(intakeLauncherController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Indexer PID", new PIDSendable(indexerController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Fuel Subsystem",
                (builder) -> builder.addStringProperty("Current State", () -> currentState.toString(), null));
        SmartDashboard.putData(
                "Fuel Subsystem/Launcher Mode",
                new SplitButtonChooser<>(
                        () -> useCustomVelocity,
                        List.of(false, true),
                        Set.of(v -> useCustomVelocity = v),
                        useCustomVelocity,
                        str -> str.equals("Custom"),
                        bool -> bool ? "Custom" : "Calculator"));
        SmartDashboard.putData(
                "Fuel Subsystem/Manual Controls",
                SplitButtonChooser.withEnum(
                        () -> dashboardFuelAction,
                        Set.of(newAction -> dashboardFuelAction = newAction),
                        dashboardFuelAction,
                        DashboardFuelAction.class));
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
            launchVelocityIntakeLauncher.set(
                    launchVelocityIntakeLauncher.get().plus(increase ? velocityChange : velocityChange.unaryMinus()));
        });
    }

    public Command temporarilyEnableManualLaunch() {
        return Commands.deferredProxy(() -> {
            if (useCustomVelocity) return Commands.none(); // already enabled, so this wouldn't do anything
            return Commands.startEnd(() -> useCustomVelocity = true, () -> useCustomVelocity = false);
        });
    }

    private Command idleCommand() {
        return run(() -> {
            currentState = FuelAction.IDLE;
            intakeLauncherController.setDutyCycle(0);
            indexerController.setDutyCycle(0);
            intakeLauncherController.setVelocity(RotationsPerSecond.zero());
            indexerController.setVelocity(RotationsPerSecond.zero());
            // System.out.println("SET VELOCITY TO 0");
        });
    }

    public Command ejectCommand() {
        return run(() -> {
            currentState = FuelAction.EJECT;
            intakeLauncherController.setVelocity(ejectVelocityIntakeLauncher.get());
            indexerController.setVelocity(ejectVelocityIndexer.get());
        });
    }

    public Command intakeCommand() {
        return run(() -> {
            currentState = FuelAction.INTAKE;
            intakeLauncherController.setVelocity(intakeVelocityIntakeLauncher.get());
            indexerController.setVelocity(intakeVelocityIndexer.get());
        });
    }

    public Command launchCommand() {
        return run(() -> {
            currentState = FuelAction.LAUNCH;
            intakeLauncherController.setVelocity(getShooterVelocity());
            indexerController.setVelocity(
                    launchVelocityIndexer.get().gt(getShooterVelocity())
                            ? getShooterVelocity()
                            : launchVelocityIndexer.get());
        });
    }

    public Command launchAll() {
        Timer timer = new Timer();
        return launchCommand().until(() -> {
            if (!timer.hasElapsed(4)) {
                return false;
            } else if (intakeLauncherController
                    .getMechanismVelocity()
                    .gt(getShooterVelocity().minus(RotationsPerSecond.of(20)))) {
                return true;
            }
            return false;
        });
    }

    public Command windUpCommand() {
        return run(() -> {
            currentState = FuelAction.WIND_UP;
            intakeLauncherController.setVelocity(getShooterVelocity());
            indexerController.setVelocity(
                    windUpVelocityIndexer.get().gt(getShooterVelocity())
                            ? getShooterVelocity()
                            : windUpVelocityIndexer.get());
        });
    }

    public Command unjamCommand() {
        return run(() -> {
            currentState = FuelAction.UNJAM;
            intakeLauncherController.setVelocity(unJamVelocityIntakeLauncher.get());
            indexerController.setVelocity(unJamVelocityIndexer.get());
        });
    }

    private AngularVelocity getShooterVelocity() {
        return useCustomVelocity
                ? launchVelocityIntakeLauncher.get()
                : shooterCalculator.calculateVelocity().velocity();
    }

    public Command windUpAndLaunchCommand() {
        return Commands.sequence(
                windUpCommand()
                        .until(() -> intakeLauncherController
                                .getMechanismVelocity()
                                .gte(getShooterVelocity().plus(FuelConstants.LAUNCH_VELOCITY_TOLERANCE)))
                        .withTimeout(FuelConstants.WINDUP_TIMEOUT),
                launchCommand());
    }

    private LinearVelocity getBallVelocity(AngularVelocity angularVelocity) {
        return MetersPerSecond.of(
                angularVelocity.in(RotationsPerSecond) * Math.PI * FuelConstants.WHEEL_RADIUS.in(Meters));
    }

    public Trigger isLaunchingTrigger() {
        return launchingTrigger;
    }

    public Trigger isIntakingTrigger() {
        return intakingTrigger;
    }

    public Trigger isEjectingTrigger() {
        return ejectingTrigger;
    }

    public Trigger isWindingUpTrigger() {
        return windingUpTrigger;
    }

    public Command addCurrentDataToShooterMap() {
        return Commands.runOnce(() -> shooterCalculator.addCurrentDataToMap(intakeLauncher.getSpeed()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        indexer.updateTelemetry();

        SmartDashboard.putString("Fuel Subsystem/Launcher Mode Text", useCustomVelocity ? "Custom" : "Calculator");
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        indexer.simIterate();
    }

    enum FuelAction {
        IDLE,
        INTAKE,
        EJECT,
        LAUNCH,
        UNJAM,
        WIND_UP
    }

    private enum DashboardFuelAction {
        IDLE,
        INTAKE,
        EJECT,
        LAUNCH,
        UNJAM
    }
}
