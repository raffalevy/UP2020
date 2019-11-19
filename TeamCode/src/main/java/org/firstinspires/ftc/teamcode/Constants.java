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
    public static final double LS_GRAB = 0.95;
    public static final double LS_OPEN = 0.65;
//    public static final double LS_OUT = 0.25;
    public static final double LS_BACK = 0.1;

    public static final double RS_GRAB = 0.1;
    public static final double RS_OPEN = 0.3;
//    public static final double RS_OUT = 0.7;
    public static final double RS_BACK = 0.6;




    private Constants() {
    }
}
