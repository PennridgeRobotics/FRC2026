package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.FuelConstants;

// https://app.readthedocs.org/projects/phoenix-documentation/downloads/pdf/latest
@NullMarked
public class FuelSubsystem extends SubsystemBase {
    private final VictorSPX intakeLauncherRoller;
    private final VictorSPX feederRoller;

    private double percentOutputIntakeLauncherRoller;
    private double percentOutputFeederRoller;

    private boolean rollersActive;

    public FuelSubsystem() {
        intakeLauncherRoller = new VictorSPX(FuelConstants.INTAKE_LAUNCHER_MOTOR_ID);
        feederRoller = new VictorSPX(FuelConstants.FEEDER_MOTOR_ID);

        final var victorConfig = new VictorSPXConfiguration();
        victorConfig.voltageCompSaturation = FuelConstants.VOLTAGE_COMPENSATION.in(Volts);
        intakeLauncherRoller.configAllSettings(victorConfig);
        intakeLauncherRoller.enableVoltageCompensation(true);
        intakeLauncherRoller.setNeutralMode(NeutralMode.Brake);
        feederRoller.configAllSettings(victorConfig);
        feederRoller.enableVoltageCompensation(true);
        feederRoller.setNeutralMode(NeutralMode.Brake);

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
                    intakeLauncherRoller.set(
                            VictorSPXControlMode.PercentOutput, percentOutputIntakeLauncherRoller / 100.0);
                    feederRoller.set(VictorSPXControlMode.PercentOutput, percentOutputFeederRoller / 100.0);
                },
                () -> {
                    intakeLauncherRoller.set(VictorSPXControlMode.PercentOutput, 0.0);
                    feederRoller.set(VictorSPXControlMode.PercentOutput, 0.0);
                });
    }

    public Trigger rollersActive() {
        return new Trigger(() -> rollersActive);
    }
}
