// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.enums.Constants.FuelConstants;
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
    private final Trigger launching;
    private final Trigger ejecting;
    private final Trigger intaking;
    private final Trigger spinningUp;

    private final SmartMotorController intakeLauncherMotorController;
    private final SmartMotorController feederMotorController;

    private final FlyWheel intakeLauncher;
    private final FlyWheel feeder;

    /** Creates a new FeederSubsystem. */
    public FuelSubsystem() {
        final var baseIntakeLauncherSMCConfig = new SmartMotorControllerConfig(this)
                .withGearing(FuelConstants.INTAKE_LAUNCHER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INTAKE_LAUNCHER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INTAKE_LAUNCHER_INVERTED)
                .withVoltageCompensation(FuelConstants.INTAKE_LAUNCHER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INTAKE_LAUNCHER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT);
        final var intakeLauncherLeftSMCConfig = baseIntakeLauncherSMCConfig
                .clone()
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0))
                .withClosedLoopController(new PIDController(0.0, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
        final var feederSMCConfig = new SmartMotorControllerConfig(this)
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0))
                .withClosedLoopController(new PIDController(0.0, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
                .withGearing(FuelConstants.FEEDER_GEARING)
                .withOpenLoopRampRate(FuelConstants.FEEDER_RAMP_RATE)
                .withMotorInverted(FuelConstants.FEEDER_INVERTED)
                .withVoltageCompensation(FuelConstants.FEEDER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.FEEDER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.FEEDER_CURRENT_LIMIT)
                .withFollowers();

        final var intakeLauncherLeftSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
        final var intakeLauncherRightSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        final var feederSparkMax = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);

        // apply config
        new SparkWrapper(intakeLauncherRightSparkMax, DCMotor.getNEO(1), baseIntakeLauncherSMCConfig);

        intakeLauncherLeftSMCConfig.withFollowers(Pair.of(intakeLauncherRightSparkMax, true));
        intakeLauncherMotorController =
                new SparkWrapper(intakeLauncherLeftSparkMax, DCMotor.getNEO(2), intakeLauncherLeftSMCConfig);
        feederMotorController = new SparkWrapper(feederSparkMax, DCMotor.getNEO(1), feederSMCConfig);

        intakeLauncher = new FlyWheel(new FlyWheelConfig(intakeLauncherMotorController)
                .withDiameter(Inches.of(4))
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH));
        feeder = new FlyWheel(new FlyWheelConfig(feederMotorController)
                .withDiameter(Inches.of(4))
                .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH));

        setDefaultCommand(run(() -> {
            currentState = FuelAction.NONE;
            intakeLauncherMotorController.setDutyCycle(0);
            feederMotorController.setDutyCycle(0);
        }));
        currentState = FuelAction.NONE;
        launching = new Trigger(() -> currentState == FuelAction.LAUNCHING);
        ejecting = new Trigger(() -> currentState == FuelAction.EJECTING);
        intaking = new Trigger(() -> currentState == FuelAction.INTAKING);
        spinningUp = new Trigger(() -> currentState == FuelAction.SPINNING_UP);

        setupSmartDashboard();
    }

    private void setupSmartDashboard() {
        SmartDashboard.putNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT);
        SmartDashboard.putData("Intake/Launcher", (builder) -> {
            builder.addDoubleProperty(
                    "Velocity", () -> intakeLauncher.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle",
                    intakeLauncherMotorController::getDutyCycle,
                    intakeLauncherMotorController::setDutyCycle);
        });
        SmartDashboard.putData("Feeder", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> feeder.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle", feederMotorController::getDutyCycle, feederMotorController::setDutyCycle);
        });
        SmartDashboard.putData(
                "Intake/Launcher PID",
                new PIDSendable(intakeLauncherMotorController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Feeder PID", new PIDSendable(feederMotorController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Fuel Subsystem",
                (builder) -> builder.addStringProperty("Current State", () -> currentState.toString(), null));
    }

    public Command eject() {
        return run(() -> {
            currentState = FuelAction.EJECTING;
            intakeLauncherMotorController.setDutyCycle(
                    -1 * SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_EJECT_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command intake() {
        return run(() -> {
            currentState = FuelAction.INTAKING;
            intakeLauncherMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT));
        });
    }

    public Command launch() {
        return run(() -> {
            currentState = FuelAction.LAUNCHING;
            intakeLauncherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command spinUp() {
        return run(() -> {
            currentState = FuelAction.SPINNING_UP;
            intakeLauncherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            feederMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching spin-up feeder value", FuelConstants.INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
        });
    }

    public Command launchSequence() {
        return Commands.sequence(spinUp().withTimeout(FuelConstants.SPIN_UP_SECONDS), launch());
    }

    public Trigger isLaunchingTrigger() {
        return launching;
    }

    public Trigger isIntakingTrigger() {
        return intaking;
    }

    public Trigger isEjectingTrigger() {
        return ejecting;
    }

    public Trigger isSpinningUpTrigger() {
        return spinningUp;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        feeder.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        feeder.simIterate();
    }

    // COMMANDS:

    /*
     * EJECT
     * INTAKE
     * LAUNCH
     * SPIN UP
     * LAUNCH SEQUENCE
     */

    enum FuelAction {
        EJECTING,
        LAUNCHING,
        INTAKING,
        SPINNING_UP,
        NONE
    };
}
