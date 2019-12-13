package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MechnumRobot {
    // --------------------------------------
    // Declare motors

    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor blMotor = null;
    DcMotor brMotor = null;

    LinearOpMode opMode;

    void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

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

    void drive(double x_stick, double y_stick, double x_right_stick, double multiplier) {
        if (Math.abs(x_stick) + Math.abs(y_stick) > Math.abs(x_right_stick)) {
            if (x_stick > 2 * y_stick) {
                flMotor.setPower(x_stick * multiplier);
                frMotor.setPower(-x_stick * multiplier);
                blMotor.setPower(-x_stick * multiplier);
                brMotor.setPower(x_stick * multiplier);
            } else {
                flMotor.setPower(y_stick * multiplier);
                frMotor.setPower(y_stick * multiplier);
                blMotor.setPower(y_stick * multiplier);
                brMotor.setPower(y_stick * multiplier);
            }
        } else {
            flMotor.setPower(x_right_stick * multiplier);
            frMotor.setPower(-x_right_stick * multiplier);
            blMotor.setPower(x_right_stick * multiplier);
            brMotor.setPower(-x_right_stick * multiplier);
        }
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

    void updateOdometry(OdometryIMU odometry) {
        odometry.update(frMotor.getCurrentPosition(), flMotor.getCurrentPosition(), blMotor.getCurrentPosition());
    }

    void driveForward(double power) {
        flMotor.setPower(power);
        blMotor.setPower(power);
        frMotor.setPower(power);
        brMotor.setPower(power);
    }

    void strafeRight(double power) {
        flMotor.setPower(power);
        blMotor.setPower(-power);
        frMotor.setPower(-power);
        brMotor.setPower(power);
    }

    void turnClockwise(double power) {
        flMotor.setPower(power);
        blMotor.setPower(power);
        frMotor.setPower(-power);
        brMotor.setPower(-power);
    }
}