package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    DcMotor liftMotor = null;
    DcMotor leadscrewMotor = null;

    Servo leftServo = null;
    Servo rightServo = null;

    LinearOpMode opMode;

    /**
     * Initialize all motors and servos
     */
    void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Initialize drive motors
        flMotor = hardwareMap.get(DcMotor.class, "FL");
        frMotor = hardwareMap.get(DcMotor.class, "FR");
        blMotor = hardwareMap.get(DcMotor.class, "BL");
        brMotor = hardwareMap.get(DcMotor.class, "BR");

        // Initialize servos
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");

        liftMotor = hardwareMap.get(DcMotor.class, "LIFT");
        leadscrewMotor = hardwareMap.get(DcMotor.class, "LEAD");

        // Set motor directions
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leadscrewMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leadscrewMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * OLD DRIVING CODE
     * Set the motors to drive; uses incorrect trigonometry.
     */
    void driveRestricted(double forwardPower, double rightPower, double clockwisePower) {
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

    public static final double ROOT_2_OVER_2 = Math.sqrt(2) / 2;

    /**
     * Corrected holonomic driving code.
     *
     * @param x_stick       The x value of the left joystick
     * @param y_stick       The y value of the left joystick
     * @param x_right_stick The x value of the left joystick
     * @param multiplier    Number to multiply the power by.
     */
    void drive(double x_stick, double y_stick, double x_right_stick, double multiplier) {

        // Rotate the stick vector by 45 degrees, in order to correspond to the diagonal omni wheels.
        double comp1 = ROOT_2_OVER_2 * x_stick;
        double comp2 = ROOT_2_OVER_2 * y_stick;

        double x_prime = comp1 - comp2;
        double y_prime = comp1 + comp2;

        // Combine the translational power and rotational power.
        double flPower = y_prime + x_right_stick;
        double frPower = x_prime + x_right_stick;
        double blPower = -x_prime + x_right_stick;
        double brPower = -y_prime + x_right_stick;

        flMotor.setPower(flPower * multiplier);
        frMotor.setPower(frPower * multiplier);
        blMotor.setPower(blPower * multiplier);
        brMotor.setPower(brPower * multiplier);
    }

    void runLeadscrewToPosition(int targetPosition) {
        if (leadscrewMotor.getCurrentPosition() > targetPosition) {
            leadscrewMotor.setPower(-1);
            while (opMode.opModeIsActive() && leadscrewMotor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            leadscrewMotor.setPower(0);
        } else if (leadscrewMotor.getCurrentPosition() < targetPosition) {
            leadscrewMotor.setPower(1);
            while (opMode.opModeIsActive() && leadscrewMotor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            leadscrewMotor.setPower(0);
        }
    }

    void updateOdometry(OdometryIMU odometry) {
        odometry.update(frMotor.getCurrentPosition(), flMotor.getCurrentPosition(), blMotor.getCurrentPosition());
    }

    void driveForward(double power) {
        flMotor.setPower(power);
        blMotor.setPower(power);
        frMotor.setPower(-power);
        brMotor.setPower(-power);
    }

    void driveRight(double power) {
        flMotor.setPower(power);
        blMotor.setPower(-power);
        frMotor.setPower(power);
        brMotor.setPower(-power);
    }

    void turnClockwise(double power) {
        flMotor.setPower(power);
        blMotor.setPower(power);
        frMotor.setPower(power);
        brMotor.setPower(power);
    }

    void goToX(double targetX, double power, OdometryIMU odometry) {
        updateOdometry(odometry);
        if (odometry.getX() < targetX) {
            driveRight(power);
            while (opMode.opModeIsActive() && odometry.getX() < targetX) {
                updateOdometry(odometry);
                Thread.yield();
                opMode.telemetry.update();
            }
            driveStop();
            updateOdometry(odometry);
        } else if (odometry.getX() > targetX) {
            driveRight(-power);
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

    public static double normalizeTheta(double theta) {
        return theta - 2 * Math.PI * Math.floor((theta + Math.PI) / 2 * Math.PI);
    }

    public static double angleDiff(double theta1, double theta2) {
        return Math.atan2(Math.sin(theta1-theta2), Math.cos(theta1-theta2));
    }

    public static final double THETA_TOLERANCE = 0.04;

//    void goClockwiseToTheta(double targetTheta, double power, OdometryIMU odometry) {
//        updateOdometry(odometry);
//        if (angleDiff(targetTheta, odometry.getTheta()) > THETA_TOLERANCE) {
//            turnClockwise(power);
//            while (opMode.opModeIsActive() && angleDiff(targetTheta, odometry.getTheta()) > THETA_TOLERANCE) {
//                updateOdometry(odometry);
//                Thread.yield();
//                opMode.telemetry.update();
//            }
//            driveStop();
//        }
//        updateOdometry(odometry);
//    }

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
}
