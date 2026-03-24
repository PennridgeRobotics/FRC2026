// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.*;
import frc.robot.util.enums.Constants.FuelConstants;
import java.util.Set;
import java.util.function.LongConsumer;
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
    private final Trigger rawReadyToLaunchTrigger;
    private final Trigger readyToLaunchTrigger;

    private final SmartMotorController intakeLauncherController;
    private final SmartMotorController indexerController;

    private final FlyWheel intakeLauncher;
    private final FlyWheel indexer;

    private OperatorFuelRequest operatorActionRequest = OperatorFuelRequest.IDLE;
    private boolean useMaxPower = false;

    private final LoggedNetworkInteger intakeLauncherUVWMeasurementPeriod = new LoggedNetworkInteger(
            "Fuel/UVW Measurement Period Intake-Launcher", FuelConstants.INTAKE_LAUNCHER_ENCODER_MEASUREMENT_PERIOD);
    private final LoggedNetworkInteger intakeLauncherUVWAverageDepth = new LoggedNetworkInteger(
            "Fuel/UVW Average Depth Intake-Launcher", FuelConstants.INTAKE_LAUNCHER_ENCODER_AVERAGE_DEPTH);
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
    private final Supplier<AngularVelocity> launchVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Launch Velocity Indexer", FuelConstants.LAUNCH_VELOCITY_INDEXER);
    private final Supplier<AngularVelocity> windUpVelocityIndexer =
            new LoggedNetworkUnit<>("Fuel/Windup Velocity Indexer", FuelConstants.WINDUP_VELOCITY_INDEXER);

    private final Supplier<Voltage> maxPowerVoltage =
            new LoggedNetworkUnit<>("Fuel/Max Power Voltage", FuelConstants.MAX_POWER_VOLTAGE);

    public FuelSubsystem(ShooterCalculator shooterCalculator, MultiMotorInfoSendable motorInfo) {
        this.shooterCalculator = shooterCalculator;
        shooterCalculator.setIsIntaking(() -> currentState == FuelAction.INTAKE);
        shooterCalculator.setIsLaunching(() -> currentState == FuelAction.LAUNCH);

        final var intakeLauncherBaseSparkMaxConfig = new SparkMaxConfig();
        intakeLauncherBaseSparkMaxConfig
                .encoder
                .uvwMeasurementPeriod(intakeLauncherUVWMeasurementPeriod.getAsInt())
                .uvwAverageDepth(intakeLauncherUVWAverageDepth.getAsInt());
        final var intakeLauncherLeftSMCConfig = new SmartMotorControllerConfig(this)
                .withVendorConfig(new SparkMaxConfig().apply(intakeLauncherBaseSparkMaxConfig))
                .withGearing(FuelConstants.INTAKE_LAUNCHER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INTAKE_LAUNCHER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INTAKE_LAUNCHER_INVERTED)
                .withVoltageCompensation(FuelConstants.INTAKE_LAUNCHER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INTAKE_LAUNCHER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT)
                .withFeedforward(new SimpleMotorFeedforward(0.11, 0.185))
                .withClosedLoopController(new PIDController(0.05, 0.0, 0.0))
                .withSimClosedLoopController(new PIDController(0, 0, 0))
                .withSimFeedforward(new SimpleMotorFeedforward(0, 0.187))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMotorInverted(true)
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
        final var followerIntakeLauncherSMCConfig = new SmartMotorControllerConfig(this)
                .withVendorConfig(new SparkMaxConfig().apply(intakeLauncherBaseSparkMaxConfig))
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
                .withDiameter(FuelConstants.FLYWHEEL_RADIUS.times(2))
                .withMOI(KilogramSquareMeters.of(0.002849))
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH));
        indexer = new FlyWheel(new FlyWheelConfig(indexerController)
                .withDiameter(FuelConstants.FLYWHEEL_RADIUS.times(2))
                .withMOI(KilogramSquareMeters.of(0.0003368))
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH));

        setDefaultCommand(Commands.defer(
                () -> {
                    final var originalRequest = operatorActionRequest;
                    final var command =
                            switch (operatorActionRequest) {
                                case IDLE -> idleCommand();
                                case INTAKE -> intakeCommand();
                                case EJECT -> ejectCommand();
                                case LAUNCH_NO_WINDUP -> launchCommand(false);
                                case LAUNCH_WINDUP -> launchCommand(true);
                                case WINDUP -> windUpCommand();
                                case UNJAM -> unjamCommand();
                            };
                    return command.until(() -> originalRequest != operatorActionRequest);
                },
                Set.of(this)));
        currentState = FuelAction.IDLE;
        launchingTrigger = new Trigger(() -> currentState == FuelAction.LAUNCH);
        ejectingTrigger = new Trigger(() -> currentState == FuelAction.EJECT);
        intakingTrigger = new Trigger(() -> currentState == FuelAction.INTAKE);
        windingUpTrigger = new Trigger(() -> currentState == FuelAction.WIND_UP);
        rawReadyToLaunchTrigger = new Trigger(() -> intakeLauncherController
                        .getMechanismVelocity()
                        .gte(getShooterVelocity().plus(FuelConstants.LAUNCH_VELOCITY_TOLERANCE)))
                .and(() -> !shooterCalculator.isUsingSOTM()
                        || shooterCalculator.calculateShotData().isReady());
        readyToLaunchTrigger = new Trigger(() -> intakeLauncherController
                        .getMechanismVelocity()
                        .gte(getShooterVelocity().plus(FuelConstants.LAUNCH_VELOCITY_TOLERANCE)))
                .debounce(0.2, DebounceType.kFalling)
                .and(() -> !shooterCalculator.isUsingSOTM()
                        || shooterCalculator.calculateShotData().isReady())
                .debounce(0.1, DebounceType.kRising);

        motorInfo.addMotor(intakeLauncherLeftSparkMax, "Intake-Launcher Left");
        motorInfo.addMotor(intakeLauncherRightSparkMax, "Intake-Launcher Right");
        motorInfo.addMotor(indexerSparkMax, "Indexer");

        final LongConsumer updateEncoder = unused -> {
            if (DriverStation.isEnabled()) {
                DriverStation.reportError("Can't update encoder settings while the robot is enabled!", false);
                return;
            }
            final var sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig
                    .encoder
                    .uvwMeasurementPeriod(intakeLauncherUVWMeasurementPeriod.getAsInt())
                    .uvwAverageDepth(intakeLauncherUVWAverageDepth.getAsInt());
            intakeLauncherLeftSparkMax.configureAsync(
                    sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            intakeLauncherRightSparkMax.configureAsync(
                    sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        };
        intakeLauncherUVWMeasurementPeriod.addListener(updateEncoder);
        intakeLauncherUVWAverageDepth.addListener(updateEncoder);

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
        // see SmartMotorControllerTelemetryConfig
        SmartDashboard.putData(
                "Intake-Launcher PID",
                new PIDSendable(intakeLauncherController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Indexer PID", new PIDSendable(indexerController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData("Fuel Subsystem", (builder) -> {
            builder.addStringProperty("Current State", () -> currentState.toString(), null);
            builder.addStringProperty(
                    "Intake-Launcher Stall Alert",
                    new FlashingColorSupplier(
                            getStallAlertTrigger(
                                    intakeLauncherController::getStatorCurrent,
                                    FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT),
                            Color.kRed,
                            Seconds.of(0.3)),
                    null);
            builder.addStringProperty(
                    "Indexer Stall Alert",
                    new FlashingColorSupplier(
                            getStallAlertTrigger(
                                    indexerController::getStatorCurrent, FuelConstants.INDEXER_CURRENT_LIMIT),
                            Color.kRed,
                            Seconds.of(0.3)),
                    null);
            builder.addStringProperty(
                    "Ready to Launch?",
                    () -> (readyToLaunchTrigger.getAsBoolean()
                                    ? Color.kLime
                                    : (rawReadyToLaunchTrigger.getAsBoolean() ? Color.kYellow : Color.kRed))
                            .toHexString(),
                    null);
        });
        SmartDashboard.putData(
                "Fuel Subsystem/Operator Request",
                SplitButtonChooser.withEnum(
                        () -> operatorActionRequest,
                        Set.of(this::setOperatorActionRequest),
                        operatorActionRequest,
                        OperatorFuelRequest.class));
    }

    private Trigger getStallAlertTrigger(Supplier<Current> statorCurrentSupplier, Current statorCurrentLimit) {
        return new Trigger(() -> statorCurrentLimit.isNear(statorCurrentSupplier.get(), Amps.of(0.5)))
                .debounce(0.5, DebounceType.kRising);
    }

    public Command requestAsOperator(OperatorFuelRequest request) {
        return Commands.run(() -> setOperatorActionRequest(request))
                .finallyDo(() -> setOperatorActionRequest(OperatorFuelRequest.IDLE));
    }

    private void setOperatorActionRequest(OperatorFuelRequest request) {
        this.operatorActionRequest = request;
    }

    public Command temporarilyUseMaxPower() {
        return Commands.startEnd(
                () -> {
                    intakeLauncherController.stopClosedLoopController();
                    indexerController.stopClosedLoopController();
                    useMaxPower = true;
                },
                () -> {
                    intakeLauncherController.startClosedLoopController();
                    indexerController.startClosedLoopController();
                    useMaxPower = false;
                });
    }

    private void setVelocityOrMaxPower(SmartMotorController motorController, AngularVelocity angularVelocity) {
        if (useMaxPower) {
            motorController.setVoltage(
                    maxPowerVoltage.get().times(Math.signum(angularVelocity.in(RotationsPerSecond))));
        } else if (motorController
                .getMechanismSetpointVelocity()
                .map(v -> !v.isEquivalent(angularVelocity))
                .orElse(true)) {
            motorController.setVelocity(angularVelocity);
        }
    }

    public Command idleCommand() {
        return run(this::reset); // System.out.println("SET VELOCITY TO 0");
    }

    private void reset() {
        currentState = FuelAction.IDLE;
        intakeLauncherController.setDutyCycle(0);
        indexerController.setDutyCycle(0);
        intakeLauncherController.setVelocity(RotationsPerSecond.zero());
        indexerController.setVelocity(RotationsPerSecond.zero());
    }

    public Command ejectCommand() {
        return run(() -> {
            currentState = FuelAction.EJECT;
            setVelocityOrMaxPower(intakeLauncherController, ejectVelocityIntakeLauncher.get());
            setVelocityOrMaxPower(indexerController, ejectVelocityIndexer.get());
        });
    }

    public Command intakeCommand() {
        return run(() -> {
            currentState = FuelAction.INTAKE;
            setVelocityOrMaxPower(intakeLauncherController, intakeVelocityIntakeLauncher.get());
            setVelocityOrMaxPower(indexerController, intakeVelocityIndexer.get());
        });
    }

    public Command launchCommand(boolean onlyWhileReady) {
        return run(() -> {
            if (onlyWhileReady && !readyToLaunchTrigger.getAsBoolean()) {
                windUp();
                return;
            }
            currentState = FuelAction.LAUNCH;
            setVelocityOrMaxPower(intakeLauncherController, getShooterVelocity());
            setVelocityOrMaxPower(
                    indexerController,
                    launchVelocityIndexer.get().gt(getShooterVelocity())
                            ? getShooterVelocity()
                            : launchVelocityIndexer.get());
        });
    }

    public Command windUpCommand() {
        return run(this::windUp);
    }

    private void windUp() {
        currentState = FuelAction.WIND_UP;
        setVelocityOrMaxPower(intakeLauncherController, getShooterVelocity());
        setVelocityOrMaxPower(indexerController, windUpVelocityIndexer.get());
    }

    public Command unjamCommand() {
        return run(() -> {
            currentState = FuelAction.UNJAM;
            setVelocityOrMaxPower(intakeLauncherController, unJamVelocityIntakeLauncher.get());
            setVelocityOrMaxPower(indexerController, unJamVelocityIndexer.get());
        });
    }

    private AngularVelocity getShooterVelocity() {
        return shooterCalculator.calculateShotData().velocity();
    }

    private LinearVelocity getBallVelocity(AngularVelocity angularVelocity) {
        return MetersPerSecond.of(
                angularVelocity.in(RotationsPerSecond) * Math.PI * FuelConstants.FLYWHEEL_RADIUS.in(Meters));
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
        intakeLauncher.updateTelemetry();
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        intakeLauncher.simIterate();
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

    public enum OperatorFuelRequest {
        IDLE,
        INTAKE,
        EJECT,
        LAUNCH_NO_WINDUP,
        LAUNCH_WINDUP,
        WINDUP,
        UNJAM
    }
}
