package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkInteger;
import frc.robot.util.dashboard.LoggedNetworkString;
import frc.robot.util.enums.Constants.FuelConstants;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import yams.motorcontrollers.SmartMotorController;

@NullMarked
public class UnjamManager extends Command {
    private final FuelSubsystem fuelSubsystem;
    private final ShooterCalculator shooterCalculator;
    private final SmartMotorController intakeLauncher;
    private final SmartMotorController indexerController;

    private final Trigger intakeLauncherStallAlertTrigger;
    private final Trigger indexerStallAlertTrigger;
    private final Trigger intakeLauncherAutoStallActionTrigger;
    private final Trigger indexerAutoStallActionTrigger;

    private int useMaxPowerIntakeLauncher = 0;
    private int useMaxPowerIndexer = 0;
    private int smartUnjamLoopCount = 0;

    private final LoggedNetworkBoolean loggedUsingSmartUnjam;
    private final LoggedNetworkBoolean loggedSmartUnjamReady;

    private final Trigger usingSmartUnjamTrigger;
    private final Trigger isReadyTrigger;

    private final Timer stageTimer = new Timer();
    private SmartUnjamStage smartUnjamStage = SmartUnjamStage.INACTIVE_0;

    public UnjamManager(
            FuelSubsystem fuelSubsystem,
            ShooterCalculator shooterCalculator,
            SmartMotorController intakeLauncher,
            SmartMotorController indexerController) {
        this.fuelSubsystem = fuelSubsystem;
        this.shooterCalculator = shooterCalculator;
        this.intakeLauncher = intakeLauncher;
        this.indexerController = indexerController;
        intakeLauncherStallAlertTrigger =
                getStallAlertTrigger(intakeLauncher::getStatorCurrent, FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT);
        indexerStallAlertTrigger =
                getStallAlertTrigger(indexerController::getStatorCurrent, FuelConstants.INDEXER_CURRENT_LIMIT);

        intakeLauncherAutoStallActionTrigger = intakeLauncherStallAlertTrigger
                .debounce(0.2, DebounceType.kRising)
                .whileTrue(temporarilyUseMaxPowerIntakeLauncher());
        indexerAutoStallActionTrigger = indexerStallAlertTrigger.whileTrue(temporarilyUseMaxPowerIndexer());

        final var rootTopicPrefix = "/Fuel/";
        final var alertsTopicPrefix = rootTopicPrefix + "Alerts/";
        loggedUsingSmartUnjam = new LoggedNetworkBoolean(rootTopicPrefix + "Using Smart Unjam", false);
        loggedSmartUnjamReady = new LoggedNetworkBoolean(rootTopicPrefix + "Smart Unjam Ready", false);
        new LoggedNetworkString(rootTopicPrefix + "Smart Unjam Stage", this::getStageLabel);
        new LoggedNetworkInteger(rootTopicPrefix + "Smart Unjam Loop Count", () -> smartUnjamLoopCount);
        new LoggedNetworkString(
                alertsTopicPrefix + "Intake-Launcher Stall Alert", () -> (intakeLauncherStallAlertTrigger.getAsBoolean()
                                ? (intakeLauncherAutoStallActionTrigger.getAsBoolean() ? Color.kRed : Color.kYellow)
                                : Color.kBlack)
                        .toHexString());
        new LoggedNetworkString(
                alertsTopicPrefix + "Indexer Stall Alert", () -> (indexerStallAlertTrigger.getAsBoolean()
                                ? (indexerAutoStallActionTrigger.getAsBoolean() ? Color.kRed : Color.kYellow)
                                : Color.kBlack)
                        .toHexString());
        new LoggedNetworkInteger(rootTopicPrefix + "Intake-Launcher Max Power Count", () -> useMaxPowerIntakeLauncher);
        new LoggedNetworkInteger(rootTopicPrefix + "Indexer Max Power Count", () -> useMaxPowerIndexer);

        usingSmartUnjamTrigger = new Trigger(loggedUsingSmartUnjam);
        isReadyTrigger = new Trigger(loggedSmartUnjamReady);
    }

    public Command temporarilyUseMaxPowerIntakeLauncher() {
        return Commands.startEnd(
                () -> {
                    if (useMaxPowerIntakeLauncher++ == 0) {
                        intakeLauncher.stopClosedLoopController();
                    }
                },
                () -> {
                    if (--useMaxPowerIntakeLauncher == 0) {
                        intakeLauncher.startClosedLoopController();
                    }
                });
    }

    public Command temporarilyUseMaxPowerIndexer() {
        return Commands.startEnd(
                () -> {
                    if (useMaxPowerIndexer++ == 0) {
                        indexerController.stopClosedLoopController();
                    }
                },
                () -> {
                    if (--useMaxPowerIndexer == 0) {
                        indexerController.startClosedLoopController();
                    }
                });
    }

    public Command temporarilyUseMaxPowerAll() {
        return Commands.parallel(temporarilyUseMaxPowerIntakeLauncher(), temporarilyUseMaxPowerIndexer());
    }

    public Command smartUnjamCommand() {
        return Commands.defer(
                () -> {
                    if (loggedUsingSmartUnjam.getAsBoolean()) {
                        DriverStation.reportError("Trying to use smart unjam while already using smart unjam!", false);
                        return Commands.none();
                    }
                    return Commands.runEnd(this::executeSmartUnjam, this::stopSmartUnjam)
                            .beforeStarting(this::startSmartUnjam);
                },
                Set.of(fuelSubsystem));
    }

    private void startSmartUnjam() {
        loggedUsingSmartUnjam.set(true);
        loggedSmartUnjamReady.set(false);
        smartUnjamLoopCount = 0;
        setStageAndResetTimer(SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1);
    }

    private void stopSmartUnjam() {
        loggedUsingSmartUnjam.set(false);
        loggedSmartUnjamReady.set(false);
        setStageAndResetTimer(SmartUnjamStage.INACTIVE_0);
    }

    private void executeSmartUnjam() {
        final boolean ready = isSmartUnjamReady();
        loggedSmartUnjamReady.set(ready);

        if (ready) {
            setStageAndResetTimer(SmartUnjamStage.NORMAL_7);
            fuelSubsystem.launch(true);
            return;
        }

        if (smartUnjamStage == SmartUnjamStage.INACTIVE_0 || smartUnjamStage == SmartUnjamStage.NORMAL_7) {
            setStageAndResetTimer(SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1);
        }

        if ((smartUnjamStage == SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1
                        || smartUnjamStage == SmartUnjamStage.INDEXER_REVERSE_LAUNCH_3
                        || smartUnjamStage == SmartUnjamStage.FORCE_LAUNCH_5)
                && isLauncherAtOrAboveSpeed(FuelConstants.SMART_UNJAM_MIN_LAUNCHER_SPEED)
                && isIndexerAtOrAboveSpeed(FuelConstants.SMART_UNJAM_MIN_INDEXER_SPEED)) {
            setStageAndResetTimer(/*SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1*/ smartUnjamStage); // almost unjammed...
        }

        runStageOutput();
        if (smartUnjamStage == SmartUnjamStage.EJECT_6) {
            if (hasStageDurationPassed()) { // eject stage should have a constant duration
                advanceStage();
            }
        } else if (isLowSpeedForStageDuration()) {
            advanceStage();
        }
    }

    private void runStageOutput() {
        switch (smartUnjamStage) {
            case INDEXER_REVERSE_LAUNCH_1, INDEXER_REVERSE_LAUNCH_3 -> {
                fuelSubsystem.setVelocityOrMaxPower(intakeLauncher, getLauncherTargetVelocity(), true);
                fuelSubsystem.setVelocityOrMaxPower(
                        indexerController, FuelConstants.UNJAM_VELOCITY_INDEXER, useMaxPowerForIndexerSmartUnjam());
            }
            case BOTH_REVERSE_PULSE_2, BOTH_REVERSE_PULSE_4 -> {
                fuelSubsystem.setVelocityOrMaxPower(intakeLauncher, FuelConstants.UNJAM_VELOCITY_INTAKE_LAUNCHER, true);
                fuelSubsystem.setVelocityOrMaxPower(
                        indexerController, FuelConstants.UNJAM_VELOCITY_INDEXER, useMaxPowerForIndexerSmartUnjam());
            }
            case FORCE_LAUNCH_5 -> {
                fuelSubsystem.setVelocityOrMaxPower(intakeLauncher, getLauncherTargetVelocity(), true);
                fuelSubsystem.setVelocityOrMaxPower(indexerController, FuelConstants.LAUNCH_VELOCITY_INDEXER, true);
            }
            case EJECT_6 -> {
                fuelSubsystem.setVelocityOrMaxPower(intakeLauncher, FuelConstants.EJECT_VELOCITY_INTAKE_LAUNCHER, true);
                fuelSubsystem.setVelocityOrMaxPower(indexerController, FuelConstants.EJECT_VELOCITY_INDEXER, true);
            }
            case NORMAL_7 -> fuelSubsystem.launch(true);
            case INACTIVE_0 -> {
                // Intentionally no output; unjam command is not actively controlling motors.
            }
        }
    }

    private boolean useMaxPowerForIndexerSmartUnjam() {
        if (useMaxPowerIndexer()) return true;
        return indexerController.getMechanismVelocity().isNear(RotationsPerSecond.zero(), 1.0);
    }

    private AngularVelocity getLauncherTargetVelocity() {
        return shooterCalculator.calculateShotData().velocity();
    }

    private boolean isSmartUnjamReady() {
        if (!loggedUsingSmartUnjam.getAsBoolean()) {
            return false;
        }
        final boolean intakeStalled = intakeLauncherStallAlertTrigger.getAsBoolean();
        final boolean indexerStalled = indexerStallAlertTrigger.getAsBoolean();
        return (smartUnjamStage == SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1
                        || smartUnjamStage == SmartUnjamStage.INDEXER_REVERSE_LAUNCH_3
                        || smartUnjamStage == SmartUnjamStage.FORCE_LAUNCH_5
                        || smartUnjamStage == SmartUnjamStage.NORMAL_7)
                && !intakeStalled
                && !indexerStalled
                && isLauncherAtOrAboveSpeed(FuelConstants.SMART_UNJAM_READY_MIN_LAUNCHER_SPEED);
    }

    private boolean isLauncherBelowSpeed(AngularVelocity threshold) {
        return intakeLauncher.getMechanismVelocity().isNear(RotationsPerSecond.zero(), threshold);
    }

    private boolean isLauncherAtOrAboveSpeed(AngularVelocity threshold) {
        return !isLauncherBelowSpeed(threshold);
    }

    private boolean isIndexerAtOrAboveSpeed(AngularVelocity threshold) {
        return !indexerController.getMechanismVelocity().isNear(RotationsPerSecond.zero(), threshold);
    }

    private boolean isLowSpeedForStageDuration() {
        return isLauncherBelowSpeed(FuelConstants.SMART_UNJAM_MIN_LAUNCHER_SPEED) && hasStageDurationPassed();
    }

    private boolean hasStageDurationPassed() {
        return stageTimer.hasElapsed(getStageDurationSeconds());
    }

    private double getStageDurationSeconds() {
        return switch (smartUnjamStage) {
            case INDEXER_REVERSE_LAUNCH_1, INDEXER_REVERSE_LAUNCH_3 ->
                FuelConstants.SMART_UNJAM_INDEXER_REVERSE_LAUNCH_TIME.in(Seconds);
            case BOTH_REVERSE_PULSE_2, BOTH_REVERSE_PULSE_4 ->
                FuelConstants.SMART_UNJAM_BOTH_REVERSE_PULSE_TIME.in(Seconds);
            case FORCE_LAUNCH_5 -> FuelConstants.SMART_UNJAM_FORCE_LAUNCH_TIME.in(Seconds);
            case EJECT_6 -> FuelConstants.SMART_UNJAM_EJECT_TIME.in(Seconds);
            case INACTIVE_0, NORMAL_7 -> Double.POSITIVE_INFINITY;
        };
    }

    private void advanceStage() {
        switch (smartUnjamStage) {
            case INDEXER_REVERSE_LAUNCH_1 -> setStageAndResetTimer(SmartUnjamStage.BOTH_REVERSE_PULSE_2);
            case BOTH_REVERSE_PULSE_2 -> setStageAndResetTimer(SmartUnjamStage.INDEXER_REVERSE_LAUNCH_3);
            case INDEXER_REVERSE_LAUNCH_3 -> setStageAndResetTimer(SmartUnjamStage.BOTH_REVERSE_PULSE_4);
            case BOTH_REVERSE_PULSE_4 -> setStageAndResetTimer(SmartUnjamStage.FORCE_LAUNCH_5);
            case FORCE_LAUNCH_5 -> {
                smartUnjamLoopCount++;
                if (smartUnjamLoopCount % 2 == 0) {
                    setStageAndResetTimer(SmartUnjamStage.EJECT_6);
                } else {
                    setStageAndResetTimer(SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1);
                }
            }
            case EJECT_6, INACTIVE_0, NORMAL_7 -> setStageAndResetTimer(SmartUnjamStage.INDEXER_REVERSE_LAUNCH_1);
        }
    }

    private void setStageAndResetTimer(SmartUnjamStage nextStage) {
        smartUnjamStage = nextStage;
        stageTimer.restart();
    }

    private String getStageLabel() {
        return switch (smartUnjamStage) {
            case INACTIVE_0 -> "0. Inactive";
            case INDEXER_REVERSE_LAUNCH_1 -> "1. Launch (Indexer Reversed)";
            case BOTH_REVERSE_PULSE_2 -> "2. Both Reversed";
            case INDEXER_REVERSE_LAUNCH_3 -> "3. Launch (Indexer Reversed)";
            case BOTH_REVERSE_PULSE_4 -> "4. Both Reversed";
            case FORCE_LAUNCH_5 -> "5. Launch (FORCE)";
            case EJECT_6 -> "6. Eject";
            case NORMAL_7 -> "7. Normal";
        };
    }

    public boolean useMaxPowerIntakeLauncher() {
        return useMaxPowerIntakeLauncher > 0;
    }

    public boolean useMaxPowerIndexer() {
        return useMaxPowerIndexer > 0;
    }

    public Trigger isUsingSmartUnjamTrigger() {
        return usingSmartUnjamTrigger;
    }

    public Trigger isReadyTrigger() {
        return isReadyTrigger;
    }

    public Trigger intakeLauncherStalledTrigger() {
        return intakeLauncherStallAlertTrigger;
    }

    public Trigger indexerStalledTrigger() {
        return indexerStallAlertTrigger;
    }

    private Trigger getStallAlertTrigger(Supplier<Current> statorCurrentSupplier, Current statorCurrentLimit) {
        return new Trigger(() -> statorCurrentLimit.isNear(statorCurrentSupplier.get(), Amps.of(1.0)))
                .debounce(0.2, DebounceType.kRising)
                .debounce(0.08, DebounceType.kFalling);
    }

    private enum SmartUnjamStage {
        INACTIVE_0,
        INDEXER_REVERSE_LAUNCH_1,
        BOTH_REVERSE_PULSE_2,
        INDEXER_REVERSE_LAUNCH_3,
        BOTH_REVERSE_PULSE_4,
        FORCE_LAUNCH_5,
        EJECT_6,
        NORMAL_7
    }
}
