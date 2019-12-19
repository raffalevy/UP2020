package org.firstinspires.ftc.teamcode;

/**
 * Constants for omni robot
 */
public final class Constants {

    // DRIVING CONSTANTS
    public static final float DRIVE_LEFT_STICK_THRESHOLD_SQUARED = .25f;
    public static final float DRIVE_POWER = 0.9f;
    public static final float DRIVE_POWER_SLOW = .35f;
    public static final float DRIVE_STICK_THRESHOLD = .35f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .65f;

    // Servo Positions

    public static final double LS_UP = 0.98;
    public static final double LS_DOWN = .6;

    public static final double RS_UP = .07;
    public static final double RS_DOWN = .45;

    private Constants() {
    }
}
