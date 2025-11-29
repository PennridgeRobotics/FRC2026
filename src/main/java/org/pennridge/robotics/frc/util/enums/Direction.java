package org.pennridge.robotics.frc.util.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public enum Direction {
    NORTH(0, 0),
    NORTHEAST(60, 45),
    EAST(90, 90),
    SOUTHEAST(120, 135),
    SOUTH(180, 180),
    SOUTHWEST(-120, 225),
    WEST(-90, 270),
    NORTHWEST(-60, 315);

    private final int angle;
    private final int compassAngle;
    private final boolean enabled = true; // might need?
    private final Rotation2d compassRotation2d;

    Direction(int angle, int compassAngle) {
        this.angle = angle;
        this.compassAngle = compassAngle;
        compassRotation2d = Rotation2d.fromDegrees(-compassAngle);
    }

    /**
     * @param x 1 for right, -1 for left
     * @param y 1 for up, -1 for down
     */
    @Nullable public static Direction fromXY(final int x, final int y) {
        if (x > 0) {
            if (y > 0) return NORTHEAST;
            if (y < 0) return SOUTHEAST;
            return EAST;
        }
        if (x == 0) {
            if (y > 0) return NORTH;
            if (y < 0) return SOUTH;
            return null;
        }
        if (y > 0) return NORTHWEST;
        if (y < 0) return SOUTHWEST;
        return WEST;
    }

    public static Direction getClosestDirection(final Rotation2d angle) {
        return getClosestDirection(-angle.getDegrees());
    }

    public static Direction getClosestDirection(final double angle) {
        final int normalizedAngle = Math.floorMod(Math.round(angle), 360);
        Direction closest = NORTH;
        int closestDiff = Integer.MAX_VALUE;
        for (final Direction direction : values()) {
            if (!direction.enabled) {
                continue;
            }
            final int dirAngle = direction.angle + 360 * (direction.angle < 0 ? 1 : 0);
            final int dirAngle2 = dirAngle + 360;
            final int diff = Math.min(Math.abs(normalizedAngle - dirAngle), Math.abs(normalizedAngle - dirAngle2));
            if (diff < closestDiff) {
                closest = direction;
                closestDiff = diff;
            }
        }
        return closest;
    }

    public static Direction getClosestCompassDirection(final Rotation2d angle, final boolean enabledOnly) {
        final double degrees = -angle.getDegrees();
        final int normalizedAngle = Math.floorMod(Math.round(degrees), 360);
        Direction closest = NORTH;
        int closestDiff = Integer.MAX_VALUE;
        for (final Direction direction : values()) {
            if (enabledOnly && !direction.enabled) {
                continue;
            }
            final int dirAngle = direction.compassAngle + 360 * (direction.compassAngle < 0 ? 1 : 0);
            final int dirAngle2 = dirAngle + 360;
            final int diff = Math.min(Math.abs(normalizedAngle - dirAngle), Math.abs(normalizedAngle - dirAngle2));
            if (diff < closestDiff) {
                closest = direction;
                closestDiff = diff;
            }
        }
        return closest;
    }

    public Direction getClockwise(final boolean enabledOnly) {
        final var nextDir = values()[(ordinal() + 1) % values().length];
        if (enabledOnly && !nextDir.isEnabled()) {
            return nextDir.getClockwise(true);
        } else {
            return nextDir;
        }
    }

    public Direction getCounterClockwise(final boolean enabledOnly) {
        final var nextDir = values()[(ordinal() - 1 + values().length) % values().length];
        if (enabledOnly && !nextDir.isEnabled()) {
            return nextDir.getCounterClockwise(true);
        } else {
            return nextDir;
        }
    }

    public int getAngle() {
        return angle;
    }

    public Rotation2d getCompassRotation2d() {
        return compassRotation2d;
    }

    public int getCompassAngle() {
        return compassAngle;
    }

    public boolean isEnabled() {
        return enabled;
    }
}
