package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.subsystems.FuelSubsystem.OperatorFuelRequest;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkInteger;
import frc.robot.util.dashboard.LoggedNetworkString;
import frc.robot.util.enums.Constants;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import yams.motorcontrollers.SmartMotorController;

@NullMarked
public class UnjamManager extends Command {
    private final FuelSubsystem fuelSubsystem;
    private final SmartMotorController intakeLauncher;
    private final SmartMotorController indexerController;

    private final Trigger intakeLauncherStallAlertTrigger;
    private final Trigger indexerStallAlertTrigger;
    private final Trigger intakeLauncherAutoStallActionTrigger;
    private final Trigger indexerAutoStallActionTrigger;
    private final Trigger intakeLauncherWasStalledTrigger;
    private final Trigger indexerWasStalledTrigger;

    private int useMaxPowerIntakeLauncher = 0;
    private int useMaxPowerIndexer = 0;

    private final LoggedNetworkBoolean loggedUsingSmartUnjam;

    public UnjamManager(
            FuelSubsystem fuelSubsystem, SmartMotorController intakeLauncher, SmartMotorController indexerController) {
        this.fuelSubsystem = fuelSubsystem;
        this.intakeLauncher = intakeLauncher;
        this.indexerController = indexerController;
        intakeLauncherStallAlertTrigger = getStallAlertTrigger(
                intakeLauncher::getStatorCurrent, Constants.FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT);
        indexerStallAlertTrigger = getStallAlertTrigger(
                indexerController::getStatorCurrent, Constants.FuelConstants.INDEXER_CURRENT_LIMIT);

        intakeLauncherAutoStallActionTrigger = intakeLauncherStallAlertTrigger
                .debounce(0.3, DebounceType.kRising)
                .whileTrue(temporarilyUseMaxPowerIntakeLauncher());
        indexerAutoStallActionTrigger = indexerStallAlertTrigger.whileTrue(temporarilyUseMaxPowerIndexer());
        intakeLauncherWasStalledTrigger = intakeLauncherStallAlertTrigger.debounce(0.35, DebounceType.kFalling);

        final var rootTopicPrefix = "/Fuel/";
        final var alertsTopicPrefix = rootTopicPrefix + "Alerts/";
        loggedUsingSmartUnjam = new LoggedNetworkBoolean(rootTopicPrefix + "Using Smart Unjam", false);
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
        return Commands.deferredProxy(() -> {
            if (loggedUsingSmartUnjam.getAsBoolean()) {
                DriverStation.reportError("Trying to use smart unjam while already using smart unjam!", false);
                return Commands.none();
            }
            final var currentRequest = new AtomicReference<>(OperatorFuelRequest.LAUNCH_WINDUP);
            final var requestTimer = new Timer();
            requestTimer.start();
            final double requestChangeMinTime = 0.6;
            final AngularVelocity minAngularVelocity = RotationsPerSecond.of(1.0);
            // 1. try shooting normally until jammed
            // 2. if
            loggedUsingSmartUnjam.set(true);
            return Commands.runEnd(
                    () -> {
                        final boolean intakeLauncherWasStalled = intakeLauncherWasStalledTrigger.getAsBoolean();
                        final boolean indexerWasStalled = indexerWasStalledTrigger.getAsBoolean();
                        if (intakeLauncherWasStalled) {
                            if (indexerWasStalled) {
                                if (requ) currentRequest.set(OperatorFuelRequest.LAUNCH_WINDUP);
                            }
                        }
                    },
                    () -> loggedUsingSmartUnjam.set(false));
        });
    }

    public boolean useMaxPowerIntakeLauncher() {
        return useMaxPowerIntakeLauncher > 0 || loggedUsingSmartUnjam.getAsBoolean();
    }

    public boolean useMaxPowerIndexer() {
        return useMaxPowerIndexer > 0 || loggedUsingSmartUnjam.getAsBoolean();
    }

    private Trigger getStallAlertTrigger(Supplier<Current> statorCurrentSupplier, Current statorCurrentLimit) {
        return new Trigger(() -> statorCurrentLimit.isNear(statorCurrentSupplier.get(), Amps.of(1.0)))
                .debounce(0.3, DebounceType.kRising)
                .debounce(0.08, DebounceType.kFalling);
    }
}
