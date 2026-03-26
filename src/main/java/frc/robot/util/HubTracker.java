package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkString;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import java.util.Objects;
import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class HubTracker {

    static {
        final var topicPrefix = "/Hub Tracker/";
        new LoggedNetworkBoolean(topicPrefix + "Active", HubTracker::isActive);
        new LoggedNetworkBoolean(topicPrefix + "Active Next", HubTracker::isActiveNext);
        new LoggedNetworkString(topicPrefix + "Current Shift", () -> Objects.toString(getCurrentShift()));
        new LoggedNetworkString(topicPrefix + "Next Shift", () -> Objects.toString(getNextShift()));
        new LoggedNetworkUnit<TimeUnit, Time>(
                topicPrefix + "Match Time", () -> Objects.requireNonNullElse(getMatchTime(), Seconds.of(-1)));
        new LoggedNetworkUnit<TimeUnit, Time>(
                topicPrefix + "Time Elapsed in Shift",
                () -> Objects.requireNonNullElse(timeElapsedInCurrentShift(), Seconds.of(-1)));
        new LoggedNetworkUnit<TimeUnit, Time>(
                topicPrefix + "Time Left in Shift",
                () -> Objects.requireNonNullElse(timeRemainingInCurrentShift(), Seconds.of(-1)));
    }

    /** Returns an the current {@link Shift}. Will return {@code null} if disabled or in between auto and teleop. */
    public static @Nullable Shift getCurrentShift() {
        final Time matchTime = getMatchTime();
        if (matchTime == null) return null;

        for (Shift shift : Shift.values()) {
            if (matchTime.lt(shift.endTime)) {
                return shift;
            }
        }
        return null;
    }

    /**
     * Returns the current {@link Time} remaining in the current shift. Will return {@code null} if disabled or in
     * between auto and teleop.
     */
    public static @Nullable Time timeRemainingInCurrentShift() {
        final var shift = getCurrentShift();
        final var matchTime = getMatchTime();
        if (shift == null || matchTime == null) return null;
        return shift.endTime.minus(matchTime);
    }

    public static @Nullable Time timeElapsedInCurrentShift() {
        final var shift = getCurrentShift();
        final var matchTime = getMatchTime();
        if (shift == null || matchTime == null) return null;
        return matchTime.minus(shift.startTime);
    }

    /** Returns the next {@link Shift}. Will return {@code null} if disabled or in between auto and teleop. */
    public static @Nullable Shift getNextShift() {
        final Time matchTime = getMatchTime();
        if (matchTime == null) return null;

        for (Shift shift : Shift.values()) {
            if (matchTime.lt(shift.startTime)) {
                return shift;
            }
        }
        return null;
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}. Will
     * return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance, Shift shift) {
        Alliance autoWinner = getAutoWinner();
        return switch (shift.activeType) {
            case BOTH -> true;
            case AUTO_WINNER -> autoWinner == alliance;
            case AUTO_LOSER -> autoWinner != null && autoWinner != alliance;
        };
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the specified {@link Alliance}. Will
     * return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Alliance alliance) {
        Shift currentShift = getCurrentShift();
        return currentShift != null && isActive(alliance, currentShift);
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the robot's {@link Alliance}. Will
     * return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive(Shift shift) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && isActive(alliance.get(), shift);
    }

    /**
     * Returns whether the hub is active during the current {@link Shift} for the robot's {@link Alliance}. Will return
     * {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActive() {
        Shift currentShift = getCurrentShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return currentShift != null && alliance.isPresent() && isActive(alliance.get(), currentShift);
    }

    /**
     * Returns whether the hub is active for the next {@link Shift} for the specified {@link Alliance}. Will return
     * {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext(Alliance alliance) {
        Shift nextShift = getNextShift();
        return nextShift != null && isActive(alliance, nextShift);
    }

    /**
     * Returns whether the hub is active during the specified {@link Shift} for the specified {@link Alliance}. Will
     * return {@code false} if disabled or in between auto and teleop.
     */
    public static boolean isActiveNext() {
        Shift nextShift = getNextShift();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return nextShift != null && alliance.isPresent() && isActive(alliance.get(), nextShift);
    }

    /**
     * Returns the {@link Alliance} that won auto as specified by the FMS/Driver Station's game specific message data.
     * Will return {@code null} if no game message or alliance is available.
     */
    public static @Nullable Alliance getAutoWinner() {
        String msg = DriverStation.getGameSpecificMessage();
        char msgChar = !msg.isEmpty() ? msg.charAt(0) : ' ';
        return switch (msgChar) {
            case 'B' -> Alliance.Blue;
            case 'R' -> Alliance.Red;
            default -> null;
        };
    }

    /**
     * Counts up from 0 to 160 seconds as match progresses. Returns null if match isn't running or if in between auto
     * and teleop
     */
    public static @Nullable Time getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) return Seconds.of(DriverStation.getMatchTime());
            return Seconds.of(20 - DriverStation.getMatchTime());
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) return Seconds.of(DriverStation.getMatchTime());
            return Seconds.of(160 - DriverStation.getMatchTime());
        }
        return null;
    }

    /**
     * Represents an alliance shift.<br>
     *
     * <h4>Values:</h4>
     *
     * <ul>
     *   <li>{@link Shift#AUTO} (0-20 sec)
     *   <li>{@link Shift#TRANSITION} (20-30 sec)
     *   <li>{@link Shift#SHIFT_1} (30-55 sec)
     *   <li>{@link Shift#SHIFT_2} (55-80 sec)
     *   <li>{@link Shift#SHIFT_3} (80-105 sec)
     *   <li>{@link Shift#SHIFT_4} (105-130 sec)
     *   <li>{@link Shift#ENDGAME} (130-160 sec)
     * </ul>
     */
    public enum Shift {
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(20, 30, ActiveType.BOTH),
        SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
        SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
        SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
        ENDGAME(130, 160, ActiveType.BOTH);

        final Time startTime;
        final Time endTime;
        final ActiveType activeType;

        Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = Seconds.of(startTime);
            this.endTime = Seconds.of(endTime);
            this.activeType = activeType;
        }
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }
}
