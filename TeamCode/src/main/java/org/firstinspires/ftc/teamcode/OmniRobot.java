package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Contains the Omni robot's hardware
 */
public class OmniRobot {

    // --------------------------------------
    // Declare motors

    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor blMotor = null;
    DcMotor brMotor = null;

    /**
     * Initialize all motors and servos
     */
    void init(HardwareMap hardwareMap) {
        // Initialize drive motors
        flMotor = hardwareMap.get(DcMotor.class, "FL");
        frMotor = hardwareMap.get(DcMotor.class, "FR");
        blMotor = hardwareMap.get(DcMotor.class, "BL");
        brMotor = hardwareMap.get(DcMotor.class, "BR");

        // Set motor directions
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set the motors to drive
     */
    void drive(double forwardPower, double rightPower, double clockwisePower) {
        forwardPower = Range.clip(forwardPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);
        clockwisePower = Range.clip(clockwisePower, -1, 1);

        flMotor.setPower(forwardPower + rightPower + clockwisePower);
        frMotor.setPower(-forwardPower + rightPower + clockwisePower);
        blMotor.setPower(forwardPower - rightPower + clockwisePower);
        brMotor.setPower(-forwardPower - rightPower + clockwisePower);
    }

    /**
     * Stop the drive motors
     */
    void driveStop() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    public static double applyThreshold(double d, double threshold) {
        if (!(d >= threshold || d <= -threshold)) {
            return 0;
        } else {
            return d;
        }
    }
}
