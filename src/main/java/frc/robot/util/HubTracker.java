package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class HubTracker {
    static {
        SmartDashboard.putData("Hub Tracker", builder -> {
            builder.addBooleanProperty("Hub Shift Active?", HubTracker::isActive, null);
            builder.addDoubleProperty(
                    "Time Elapsed in Shift",
                    () -> {
                        final var timeElapsed = timeElapsedInCurrentShift();
                        return timeElapsed != null ? timeElapsed.in(Seconds) : -1;
                    },
                    null);
            builder.addDoubleProperty(
                    "Time Left in Shift",
                    () -> {
                        final var timeRemaining = timeRemainingInCurrentShift();
                        return timeRemaining != null ? timeRemaining.in(Seconds) : -1;
                    },
                    null);
        });
    }

    /** Returns an the current {@link Shift}. Will return {@code null} if disabled or in between auto and teleop. */
    public static @Nullable Shift getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) return null;

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.endTime) {
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
        if (shift == null) return null;
        return Seconds.of(shift.endTime - getMatchTime());
    }

    public static @Nullable Time timeElapsedInCurrentShift() {
        final var shift = getCurrentShift();
        if (shift == null) return null;
        return Seconds.of(getMatchTime() - shift.startTime);
    }

    /** Returns the next {@link Shift}. Will return {@code null} if disabled or in between auto and teleop. */
    public static @Nullable Shift getNextShift() {
        double matchTime = getMatchTime();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.startTime) {
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
     * Counts up from 0 to 160 seconds as match progresses. Returns -1 if not match isn't running or if in between auto
     * and teleop
     */
    public static double getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 20 - DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 160 - DriverStation.getMatchTime();
        }
        return -1;
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

        final int startTime;
        final int endTime;
        final ActiveType activeType;

        private Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }
}
