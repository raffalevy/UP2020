package org.firstinspires.ftc.teamcode;

/**
 * Constants for omni robot
 */
public final class Constants {

    // DRIVING CONSTANTS
    public static final float DRIVE_LEFT_STICK_THRESHOLD_SQUARED = .25f;
    public static final float DRIVE_POWER = 1;
    public static final float DRIVE_POWER_SLOW = .5f;
    public static final float DRIVE_STICK_THRESHOLD = .6f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .65f;

    // Servo positions
    public static final double LS_GRAB = 0.975;
    public static final double LS_PARALLEL = 0.875;
    public static final double LS_PERP = 0.45;
    public static final double LS_BACK = 0.15;

    public static final double RS_GRAB = 0.075;
    public static final double RS_PARALLEL = 0.175;
    public static final double RS_PERP = 0.55;
    public static final double RS_BACK = 0.85;




    private Constants() {
    }
}
