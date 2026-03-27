/*
 * ShotLUT.java - Distance-keyed lookup table for shot parameters
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

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Distance-keyed lookup table for RPM, hood angle, and TOF. Wraps WPILib's InterpolatingTreeMap so all three stay in
 * sync at any queried distance.
 *
 * <p>If you're using separate InterpolatingDoubleTreeMaps for RPM and TOF, this keeps them from drifting apart.
 *
 * <p>Usage:
 *
 * <pre>
 *   ShotLUT lut = new ShotLUT();
 *   lut.put(1.0, 2000, 45.0, 0.45);
 *   lut.put(2.0, 2800, 42.0, 0.62);
 *   lut.put(3.0, 3500, 38.0, 0.78);
 *
 *   ShotParameters shot = lut.get(1.5); // interpolates all three fields
 *   double rpm = shot.rpm();
 *   double angle = shot.angleDeg();
 * </pre>
 */
public class ShotLUT {

    private final InterpolatingTreeMap<Double, ShotParameters> map;
    private int entryCount = 0;

    public ShotLUT() {
        map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters.interpolator());
    }

    /** Insert or overwrite the parameters at a given distance. */
    public void put(double distanceM, ShotParameters params) {
        map.put(distanceM, params);
        entryCount++;
    }

    /** Convenience: insert from individual fields. */
    public void put(double distanceM, double rpm, double angleDeg, double tofSec) {
        put(distanceM, new ShotParameters(rpm, angleDeg, tofSec));
    }

    /** Get the interpolated parameters at a distance. Returns ShotParameters.ZERO if the map is empty. */
    public ShotParameters get(double distanceM) {
        ShotParameters result = map.get(distanceM);
        return result != null ? result : ShotParameters.ZERO;
    }

    public double getRPM(double distanceM) {
        return get(distanceM).rpm();
    }

    public double getAngle(double distanceM) {
        return get(distanceM).angleDeg();
    }

    public double getTOF(double distanceM) {
        return get(distanceM).tofSec();
    }

    public void clear() {
        map.clear();
        entryCount = 0;
    }

    /** Approximate number of entries added (may overcount if same key is put twice). */
    public int size() {
        return entryCount;
    }
}
