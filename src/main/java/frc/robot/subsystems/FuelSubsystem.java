// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.FuelConstants;
import java.util.List;
import java.util.Set;
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

    private boolean useCustomVelocity;
    private AngularVelocity customVelocity = RotationsPerSecond.zero();

    public FuelSubsystem(ShooterCalculator shooterCalculator) {
        this.shooterCalculator = shooterCalculator;

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
        final var indexerSMCConfig = new SmartMotorControllerConfig(this)
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0))
                .withClosedLoopController(new PIDController(0.0, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
                .withGearing(FuelConstants.INDEXER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INDEXER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INDEXER_INVERTED)
                .withVoltageCompensation(FuelConstants.INDEXER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INDEXER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INDEXER_CURRENT_LIMIT)
                .withFollowers();

        final var intakeLauncherLeftSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
        final var intakeLauncherRightSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        final var indexerSparkMax = new SparkMax(FuelConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        // apply config
        new SparkWrapper(intakeLauncherRightSparkMax, DCMotor.getNEO(1), baseIntakeLauncherSMCConfig);

        intakeLauncherLeftSMCConfig.withFollowers(Pair.of(intakeLauncherRightSparkMax, true));
        intakeLauncherController =
                new SparkWrapper(intakeLauncherLeftSparkMax, DCMotor.getNEO(2), intakeLauncherLeftSMCConfig);
        indexerController = new SparkWrapper(indexerSparkMax, DCMotor.getNEO(1), indexerSMCConfig);

        intakeLauncher = new FlyWheel(new FlyWheelConfig(intakeLauncherController)
                .withDiameter(Inches.of(4))
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH));
        indexer = new FlyWheel(new FlyWheelConfig(indexerController)
                .withDiameter(Inches.of(4))
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH));

        setDefaultCommand(run(() -> {
            currentState = FuelAction.NONE;
            intakeLauncherController.setDutyCycle(0);
            indexerController.setDutyCycle(0);
        }));
        currentState = FuelAction.NONE;
        launchingTrigger = new Trigger(() -> currentState == FuelAction.LAUNCHING);
        ejectingTrigger = new Trigger(() -> currentState == FuelAction.EJECTING);
        intakingTrigger = new Trigger(() -> currentState == FuelAction.INTAKING);
        windingUpTrigger = new Trigger(() -> currentState == FuelAction.WINDING_UP);

        setupSmartDashboard();
    }

    private void setupSmartDashboard() {
        SmartDashboard.putData("Intake/Launcher", (builder) -> {
            builder.addDoubleProperty(
                    "Velocity", () -> intakeLauncher.getSpeed().in(RotationsPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle", intakeLauncherController::getDutyCycle, intakeLauncherController::setDutyCycle);
        });
        SmartDashboard.putData("Indexer", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> indexer.getSpeed().in(RotationsPerSecond), null);
            builder.addDoubleProperty("DutyCycle", indexerController::getDutyCycle, indexerController::setDutyCycle);
        });
        SmartDashboard.putData(
                "Intake/Launcher PID",
                new PIDSendable(intakeLauncherController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Indexer PID", new PIDSendable(indexerController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData("Fuel Subsystem", (builder) -> {
            builder.addStringProperty("Current State", () -> currentState.toString(), null);
            builder.addDoubleProperty(
                    "Shooter Velocity",
                    () -> customVelocity.in(RotationsPerSecond),
                    v -> customVelocity = RotationsPerSecond.of(v));
        });
        SmartDashboard.putData(
                "Fuel Subsystem/Launcher Mode",
                new SplitButtonChooser<>(
                        () -> useCustomVelocity,
                        List.of(false, true),
                        Set.of(v -> useCustomVelocity = v),
                        useCustomVelocity,
                        str -> str.equals("Custom"),
                        bool -> bool ? "Custom" : "Calculator"));
    }

    public Command eject() {
        return run(() -> {
            currentState = FuelAction.EJECTING;
            intakeLauncherController.setVelocity(FuelConstants.EJECT_VELOCITY_INTAKE_LAUNCHER);
            indexerController.setVelocity(FuelConstants.EJECT_VELOCITY_INDEXER);
        });
    }

    public Command intake() {
        return run(() -> {
            currentState = FuelAction.INTAKING;
            intakeLauncherController.setVelocity(FuelConstants.INTAKE_VELOCITY_INTAKE_LAUNCHER);
            indexerController.setVelocity(FuelConstants.INTAKE_VELOCITY_INDEXER);
        });
    }

    public Command launch() {
        return run(() -> {
            currentState = FuelAction.LAUNCHING;
            intakeLauncherController.setVelocity(getShooterVelocity());
            indexerController.setVelocity(FuelConstants.LAUNCH_VELOCITY_INDEXER);
        });
    }

    public Command windUp() {
        return run(() -> {
            currentState = FuelAction.WINDING_UP;
            intakeLauncherController.setVelocity(getShooterVelocity());
            indexerController.setVelocity(FuelConstants.WINDUP_VELOCITY_INDEXER);
        });
    }

    private AngularVelocity getShooterVelocity() {
        return useCustomVelocity
                ? customVelocity
                : shooterCalculator.calculateVelocity().velocity();
    }

    public Command windUpAndLaunch() {
        return Commands.sequence(
                windUp().until(() -> intakeLauncherController
                        .getMechanismVelocity()
                        .gte(getShooterVelocity().plus(FuelConstants.LAUNCH_VELOCITY_TOLERANCE))),
                launch());
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
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        indexer.simIterate();
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
        WINDING_UP,
        NONE
    };
}
