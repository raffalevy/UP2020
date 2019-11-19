package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Position tracking using encoders and IMU
 */
@SuppressWarnings("WeakerAccess")
public class OdometryIMU {
    // Encoder constants
    public static final double CM_PER_TICK_A = -100.0 / 1673.0;
    public static final double CM_PER_TICK_B = -100.0 / 1693.0;
    public static final double CM_PER_TICK_C = -100.0 / 1434.0;

    // Offset to account for the omni wheels being diagonal
    public static final double STARTING_THETA = Math.PI / 4;

    // The current position and orientation of the robot
    private double x = 0;
    private double y = 0;
    private double theta = Math.PI / 4;

    // The angle at which the robot was facing when the OpMode started
    public float startingAngle = 0;

    // The previous encoder values
    private double oldA = 0;
    private double oldB = 0;
    private double oldC = 0;

    // The IMU handler - tracks the robot's orientation
    public IMU imu = new IMU();

    /**
     * Should be called in when the OpMode is initialized
     */
    public void init(HardwareMap hardwareMap) {
        imu.initIMU(hardwareMap);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    /**
     * Should be called when the OpMode is started
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */
    public void start(double a, double b, double c) {
        imu.update();
        startingAngle = imu.getZAngle();
        x = 0;
        y = 0;
        theta = STARTING_THETA;
        oldA = a;
        oldB = b;
        oldC = c;
    }

    /**
     * Updates the position and orientation based on new encoder values and the IMU
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */
    public void update(double a, double b, double c) {
        // Calculate the differences between the new and old encoder values
        // and convert them to centimeters.
        double dA = (a - oldA) * CM_PER_TICK_A;
        double dB = (b - oldB) * CM_PER_TICK_B;
        double dC = (c - oldC) * CM_PER_TICK_C;

        oldA = a;
        oldB = b;
        oldC = c;

        // Update the orientation using data from the IMU
        imu.update();
        theta = imu.getZAngle() - startingAngle + STARTING_THETA;

        // Update the total displacement using the orientation and encoder displacements
        double dx = ((dA - 2 * dB + dC) * Math.cos(theta) - (dA - dC) * Math.sin(theta)) / 2;
        double dy = ((dA - 2 * dB + dC) * Math.sin(theta) + (dA - dC) * Math.cos(theta)) / 2;

        // Add the displacement to the total position
        this.x += dx;
        this.y += dy;
    }

}
