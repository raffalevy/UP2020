package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Code for driving the new Mecanum robot
 *
 * December 2019
 */
public class MecanumRobot {
    // --------------------------------------
    // Declare motors

    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor blMotor = null;
    DcMotor brMotor = null;
    DcMotor liftMotor = null;

    Servo leftServo = null;
    Servo rightServo = null;

    LinearOpMode opMode;

    void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Initialize drive motors
        flMotor = hardwareMap.get(DcMotor.class, "FL");
        frMotor = hardwareMap.get(DcMotor.class, "FR");
        blMotor = hardwareMap.get(DcMotor.class, "BL");
        brMotor = hardwareMap.get(DcMotor.class, "BR");

        liftMotor = hardwareMap.get(DcMotor.class, "LIFT");

        // Initialize servos
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");

        // Set motor directions
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Decide whether to turn or strafe depending on the angle of the joysticks
     */
    void drive(double x_stick, double y_stick, double x_right_stick, double multiplier) {
            if (Math.abs(x_stick) > (2 * Math.abs(y_stick))) {
                flMotor.setPower(x_stick * multiplier);
                frMotor.setPower(-x_stick * multiplier);
                blMotor.setPower(-x_stick * multiplier);
                brMotor.setPower(x_stick * multiplier);
            } else {
                flMotor.setPower((y_stick + x_right_stick) * multiplier);
                frMotor.setPower((y_stick - x_right_stick) * multiplier);
                blMotor.setPower((y_stick + x_right_stick) * multiplier);
                brMotor.setPower((y_stick - x_right_stick) * multiplier);
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
    public static double XY_TOLERANCE = .5;

    void goToXY(double targetX, double targetY, double power, OdometryIMU odemetry){
        double deltaX = odemetry.getX() - targetX;
        double deltaY = odemetry.getY() - targetY;
        double theta = Math.atan2(deltaX, deltaY);
        goToTheta(theta,.3, odemetry, new ElapsedTime(), 1 + (deltaX + deltaY/5));
        while (opMode.opModeIsActive() && Math.abs(deltaX)> XY_TOLERANCE && Math.abs(deltaY)> XY_TOLERANCE){
            updateOdometry(odemetry);
            driveForward(power);
            opMode.telemetry.update();
        }
        driveStop();
        updateOdometry(odemetry);
    }

    public static double angleDiff(double theta1, double theta2) {
        return Math.atan2(Math.sin(theta1-theta2), Math.cos(theta1-theta2));
    }

    void goToTheta(double targetTheta, double power, OdometryIMU odometry, ElapsedTime runtime, double timeout) {
        updateOdometry(odometry);
        if (angleDiff(odometry.getTheta(), targetTheta) > THETA_TOLERANCE) {
            turnClockwise(power);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && angleDiff(odometry.getTheta(), targetTheta) > THETA_TOLERANCE) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        } else if (angleDiff(odometry.getTheta(), targetTheta) < -THETA_TOLERANCE) {
            turnClockwise(-power);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && angleDiff(odometry.getTheta(), targetTheta) < -THETA_TOLERANCE) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
        }
            driveStop();
            updateOdometry(odometry);
        }
    }


    public static final double THETA_TOLERANCE = 0.04;

    void goToX(double targetX, double power, OdometryIMU odometry) {
        updateOdometry(odometry);
        if (odometry.getX() < targetX) {
            driveForward(power);
            while (opMode.opModeIsActive() && odometry.getX() < targetX) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        } else if (odometry.getX() > targetX) {
            driveForward(power);
            while (opMode.opModeIsActive() && odometry.getX() > targetX) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        }
    }
    void goToY(double targetY, double power, OdometryIMU odometry) {
        updateOdometry(odometry);
        if (odometry.getY() < targetY) {
            driveForward(power);
            while (opMode.opModeIsActive() && odometry.getY() < targetY) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        } else if (odometry.getY() > targetY) {
            driveForward(-power);
            while (opMode.opModeIsActive() && odometry.getY() > targetY) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        }
    }

}