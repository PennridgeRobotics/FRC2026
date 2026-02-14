package org.pennridge.robotics.frc.vision;

import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public interface Camera {
    @Nullable PoseEstimate update();
}
