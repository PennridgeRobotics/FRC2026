/*
 * ShotParameters.java - Shot parameters (RPM, angle, TOF) for one LUT entry
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.robot.util.lib.frcfirecontrol;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Ballistic parameters at a single distance: RPM, hood angle, and time-of-flight. All three interpolate linearly so the
 * LUT gives smooth values between data points.
 *
 * <p>For fixed-angle shooters, every entry shares the same angleDeg. Once you've got an adjustable hood, the angle
 * varies per-distance and interpolation covers the gaps.
 */
public record ShotParameters(double rpm, double angleDeg, double tofSec) {

    /** All zeros. Safe as a default when the LUT is empty. */
    public static final ShotParameters ZERO = new ShotParameters(0, 0, 0);

    /**
     * Linear interpolator for use with WPILib's InterpolatingTreeMap. Plug this into the constructor alongside
     * InverseInterpolator.forDouble().
     */
    public static Interpolator<ShotParameters> interpolator() {
        return (start, end, t) -> new ShotParameters(
                MathUtil.interpolate(start.rpm, end.rpm, t),
                MathUtil.interpolate(start.angleDeg, end.angleDeg, t),
                MathUtil.interpolate(start.tofSec, end.tofSec, t));
    }
}
