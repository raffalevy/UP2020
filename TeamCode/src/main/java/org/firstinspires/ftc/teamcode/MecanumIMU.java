package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Position tracking using encoders and IMU
 */
@SuppressWarnings("WeakerAccess")
public class MecanumIMU {

    // Offset to account for the omni wheels being diagonal
    public static final double STARTING_THETA = Math.PI / 4;

    // The current position and orientation of the robot
    private double theta = Math.PI / 4;

    // The angle at which the robot was facing when the OpMode started
    public float startingAngle = 0;

    // The IMU handler - tracks the robot's orientation
    public IMU imu = new IMU();

    /**
     * Should be called in when the OpMode is initialized
     */
    public void init(HardwareMap hardwareMap) {
        imu.initIMU(hardwareMap);
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
        theta = STARTING_THETA;
    }

    /**
     * Updates the position and orientation based on new encoder values and the IMU
     *
     * @param a The current front-right encoder value
     * @param b The current front-left encoder value
     * @param c The current back-left encoder value
     */
    public void update() {
        // Update the orientation using data from the IMU
        imu.update();
        theta = imu.getZAngle() - startingAngle + STARTING_THETA;
    }

}
