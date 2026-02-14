package org.pennridge.robotics.frc.util.enums;

public enum DriveMode {
    NORMAL("Normal"),
    BUMP_LOCK("Bump Lock"),
    ;

    private final String friendlyName;

    DriveMode(String friendlyName) {
        this.friendlyName = friendlyName;
    }

    public String getFriendlyName() {
        return friendlyName;
    }
}
