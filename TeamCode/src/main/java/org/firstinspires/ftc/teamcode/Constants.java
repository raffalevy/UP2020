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

    // Servo Positions

    public static final double LS_UP = 0.4;
    public static final double LS_DOWN = .05;

    public static final double RS_UP = .75;
    public static final double RS_DOWN = .1;

    public static final double HOOK_UP = .5;
    public static final double HOOK_DOWN = .1;

    //Encoder Constants

    public static final int flConstant1 = 0;
    public static final int flConstant2 = 0;
    public static final int flConstant3 = 0;

    public static final int frConstant1 = 0;
    public static final int frConstant2 = 0;

    public static final int blConstant1 = 0;


    private Constants() {
    }
}
