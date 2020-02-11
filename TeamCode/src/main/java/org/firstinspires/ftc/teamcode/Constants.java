package org.firstinspires.ftc.teamcode;

/**
 * Constants for robot
 */
public final class Constants {

    // DRIVING CONSTANTS
    public static final float DRIVE_LEFT_STICK_THRESHOLD_SQUARED = .25f;
    public static final float DRIVE_POWER = .8f;
    public static final float DRIVE_POWER_SLOW = .5f;
    public static final float DRIVE_STICK_THRESHOLD = .6f;
    public static final float DRIVE_STICK_THRESHOLD_SQUARED = DRIVE_STICK_THRESHOLD * DRIVE_STICK_THRESHOLD;
    public static final float TRIGGER_THRESHOLD = .65f;

    // Servo Positions

    public static final double LS_UP = 0.6;
    public static final double LS_DOWN = .32;

    public static final double RS_UP = .72;
    public static final double RS_DOWN = .95;

    public static final double HOOK1_UP = .63;
    public static final double HOOK1_DOWN = .18;

    public static final double HOOK2_UP = .0;
    public static final double HOOK2_DOWN = .8;

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
