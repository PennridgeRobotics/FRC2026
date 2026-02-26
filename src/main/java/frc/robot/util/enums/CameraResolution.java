package frc.robot.util.enums;

import org.jspecify.annotations.NullMarked;

@NullMarked
public enum CameraResolution {
    W_1920_H_1440(1920, 1440),
    W_1280_H_960(1280, 960),
    W_960_H_720(960, 720),
    W_640_H_480(640, 480),
    W_480_H_360(480, 360),
    ;

    private final int width;
    private final int height;

    CameraResolution(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }
}
