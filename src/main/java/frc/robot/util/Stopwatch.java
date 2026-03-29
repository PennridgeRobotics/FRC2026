package frc.robot.util;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class Stopwatch {
    private final String name;
    private final double startTime;
    private double lastTime;
    private double minTimeToLog;

    public Stopwatch(String name) {
        this(name, Milliseconds.of(5));
    }

    public Stopwatch(String name, @Nullable Time minimumTimeToLog) {
        this.name = name;
        startTime = Timer.getFPGATimestamp();
        lastTime = startTime;
        minTimeToLog = minimumTimeToLog == null ? 0.0 : minimumTimeToLog.in(Seconds);
    }

    public void logTime(String name) {
        final var currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastTime >= minTimeToLog) {
            System.out.printf(
                    "[%s] %s took %.1fms (Total: %.1fms)\n",
                    this.name, name, (currentTime - lastTime) * 1000, (currentTime - startTime) * 1000);
        }
        lastTime = currentTime;
    }

    public void logFinalTime() {
        final var currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime >= minTimeToLog) {
            System.out.printf("[%s] Took %.1fms in total\n", this.name, (currentTime - startTime) * 1000);
        }
    }
}
