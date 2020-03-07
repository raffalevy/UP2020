package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.HOOK1_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.HOOK1_UP;
import static org.firstinspires.ftc.teamcode.Constants.HOOK2_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.HOOK2_UP;
import static org.firstinspires.ftc.teamcode.Constants.LS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.LS_UP;
import static org.firstinspires.ftc.teamcode.Constants.RS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.RS_UP;

/**
 * Code for driving the new Mecanum robot
 * <p>
 * December 2019
 */
public class MecanumRobot {
    // --------------------------------------
    // Declare motors

    public static final double THETA_TOLERANCE = 0.04;
    public static double XY_TOLERANCE = 0.05;
    DcMotor flMotor = null;
    DcMotor frMotor = null;
    DcMotor blMotor = null;
    DcMotor brMotor = null;
    DcMotor liftMotor = null;
    Servo leftServo = null;
    Servo rightServo = null;
    Servo hookServo1 = null;
    Servo hookServo2 = null;
    LinearOpMode opMode;

    public static double angleDiff(double theta1, double theta2) {
        return Math.atan2(Math.sin(theta1 - theta2), Math.cos(theta1 - theta2));
    }

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
        hookServo1 = hardwareMap.get(Servo.class, "HOOK1");
        hookServo2 = hardwareMap.get(Servo.class, "HOOK2");

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
        if (Math.abs(x_stick) >= (2 * Math.abs(y_stick)) + .1) {
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

    void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void hooksDown() {
        hookServo1.setPosition(HOOK1_DOWN);
        hookServo2.setPosition(HOOK2_DOWN);
    }

    void hooksUp() {
        hookServo1.setPosition(HOOK1_UP);
        hookServo2.setPosition(HOOK2_UP);
    }
    void grab (){
        leftServo.setPosition(LS_DOWN);
        rightServo.setPosition(RS_DOWN);
    }
    void release (){
        leftServo.setPosition(LS_UP);
        rightServo.setPosition(RS_UP);
    }
    void driveForwardByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            driveForward(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            driveForward(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }
    void strafeRightByEncoder(int positionChange, DcMotor motor, double power) {
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            strafeRight(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            strafeRight(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }
    void turnClockwiseByEncoder (int positionChange, DcMotor motor, double power){
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        int targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            turnClockwise(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            turnClockwise(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }
    }

    void goToTheta(double targetTheta, double power, MecanumOdometry odometry, ElapsedTime runtime, double timeout) {
        odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        if (angleDiff(odometry.getTheta(), targetTheta) > THETA_TOLERANCE) {
            turnClockwise(power);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && angleDiff(odometry.getTheta(), targetTheta) > THETA_TOLERANCE) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        } else if (angleDiff(odometry.getTheta(), targetTheta) < -THETA_TOLERANCE) {
            turnClockwise(-power);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && angleDiff(odometry.getTheta(), targetTheta) < -THETA_TOLERANCE) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        }
    }

    void strafeToX(double targetX, double power, MecanumOdometry odometry, ElapsedTime runtime, double timeout){
        int forwardsBackwardsFactor;
        if (odometry.getTheta() > Math.PI/2 && odometry.getTheta() < 3 * Math.PI / 2){
            goToTheta(Math.PI,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = -1;
        } else {
            goToTheta(0,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = 1;
        }
        odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        if (odometry.getX() < targetX){
            strafeRight(power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getX() < targetX) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        } else if (odometry.getX() > targetX){
            strafeRight(-power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getX() > targetX) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        }
    }

    void strafeToY(double targetY, double power, MecanumOdometry odometry, ElapsedTime runtime, double timeout){
        int forwardsBackwardsFactor;
        if (odometry.getTheta() > Math.PI && odometry.getTheta() < 2 * Math.PI){
            goToTheta(3 * Math.PI / 2,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = 1;
        } else {
            goToTheta(Math.PI / 2,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = -1;
        }
        odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        if (odometry.getY() < targetY){
            strafeRight(power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getY() < targetY) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        } else if (odometry.getY() > targetY){
            strafeRight(-power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getY() > targetY) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        }
    }
    void driveToX(double targetX, double power, MecanumOdometry odometry, ElapsedTime runtime, double timeout){
        int forwardsBackwardsFactor;
        if (odometry.getTheta() > Math.PI && odometry.getTheta() < 2 * Math.PI){
            goToTheta(3 * Math.PI / 2,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = 1;
        } else {
            goToTheta(Math.PI / 2,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = -1;
        }
        odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        if (odometry.getX() < targetX){
            driveForward(power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getX() < targetX) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        } else if (odometry.getX() > targetX){
            driveForward(-power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getX() > targetX) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        }
    }
    void driveToY(double targetY, double power, MecanumOdometry odometry, ElapsedTime runtime, double timeout){
        int forwardsBackwardsFactor;
        if (odometry.getTheta() > Math.PI/2 && odometry.getTheta() < 3 * Math.PI / 2){
            goToTheta(Math.PI,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = -1;
        } else {
            goToTheta(0,.7,odometry, runtime , timeout);
            forwardsBackwardsFactor = 1;
        }
        odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        if (odometry.getY() < targetY){
            driveForward(power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getY() < targetY) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        } else if (odometry.getY() > targetY){
            driveForward(-power * forwardsBackwardsFactor);
            double startTime = runtime.seconds();
            while (opMode.opModeIsActive() && runtime.seconds() - startTime < timeout && odometry.getY() > targetY) {
                odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            odometry.update(frMotor.getCurrentPosition(),flMotor.getCurrentPosition(),blMotor.getCurrentPosition());
        }
    }
    void correctedStrafeRight(double power, MecanumOdometry odometry, ElapsedTime runtime){
        double startingtheta = odometry.getTheta();
        flMotor.setPower(power);
        blMotor.setPower(-power);
        frMotor.setPower(-power);
        brMotor.setPower(power);
        goToTheta(startingtheta, 1, odometry, runtime, 1);
    }
}