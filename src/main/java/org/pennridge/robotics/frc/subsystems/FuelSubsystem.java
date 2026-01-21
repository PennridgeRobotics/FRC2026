package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.FuelConstants;

// https://app.readthedocs.org/projects/phoenix-documentation/downloads/pdf/latest
@NullMarked
public class FuelSubsystem extends SubsystemBase {
    private final SparkMax intakeLauncherRoller;
    private final SparkMax feederRoller;

    private double percentOutputIntakeLauncherRoller;
    private double percentOutputFeederRoller;

    private boolean rollersActive;

    public FuelSubsystem() {
        intakeLauncherRoller = new SparkMax(FuelConstants.INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        feederRoller = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);

        final var intakeLauncherConfig = new SparkMaxConfig();
        intakeLauncherConfig.voltageCompensation(FuelConstants.VOLTAGE_COMPENSATION.in(Volts));
        intakeLauncherConfig.smartCurrentLimit((int) Math.round(FuelConstants.MOTOR_CURRENT_LIMIT.in(Amps)));
        intakeLauncherRoller.configure(
                intakeLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        final var feederConfig = new SparkMaxConfig();
        feederConfig.voltageCompensation(FuelConstants.VOLTAGE_COMPENSATION.in(Volts));
        feederConfig.smartCurrentLimit((int) Math.round(FuelConstants.MOTOR_CURRENT_LIMIT.in(Amps)));
        feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(runRollers().onlyWhile(rollersActive()));

        SmartDashboard.putData("Fuel Subsystem", builder -> {
            builder.addDoubleProperty(
                    "Intake/Launcher Roller Output % [-100, 100]",
                    () -> percentOutputIntakeLauncherRoller, (v) -> percentOutputIntakeLauncherRoller = v);
            builder.addDoubleProperty(
                    "Feeder Roller Output % [-100, 100]",
                    () -> percentOutputFeederRoller, (v) -> percentOutputFeederRoller = v);
            builder.addBooleanProperty("Rollers Active", () -> rollersActive, (v) -> rollersActive = v);
        });
    }

    public Command runRollers() {
        return runEnd(
                () -> {
                    intakeLauncherRoller.set(percentOutputIntakeLauncherRoller / 100.0);
                    feederRoller.set(percentOutputFeederRoller / 100.0);
                },
                () -> {
                    stopRollers();
                });
    }

    private void stopRollers() {
        intakeLauncherRoller.set(0.0);
        feederRoller.set(0.0);
    }

    public Trigger rollersActive() {
        return new Trigger(() -> rollersActive);
    }

    private long lastTime = -1;

    @Override
    public void periodic() {
        if (Math.round(Timer.getFPGATimestamp()) != lastTime) {
            lastTime = Math.round(Timer.getFPGATimestamp());
            // Any debug messages?
        }
    }
}
