package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
@Disabled
@Autonomous (name = "BlueFAuto", group = "BlueAuto")
public class BlueFoundationAuto extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    OmniRobot rb = new OmniRobot();

    OdometryIMU odometry = new OdometryIMU();

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, this);

        rb.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoder(rb.frMotor);
        resetEncoder(rb.flMotor);
        resetEncoder(rb.blMotor);
        resetEncoder(rb.brMotor);

        odometry.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("x", new Func<Object>() {
            @Override
            public Object value() {
                return odometry.getX();
            }
        });
        telemetry.addData("y", new Func<Object>() {
            @Override
            public Object value() {
                return odometry.getY();
            }
        });
        telemetry.addData("theta", new Func<Object>() {
            @Override
            public Object value() {
                return odometry.getTheta();
            }
        });
        telemetry.addData("sth", new Func<Object>() {

            @Override
            public Object value() {
                return odometry.startingAngle;
            }
        });
        telemetry.addData("zzz", new Func<Object>() {

            @Override
            public Object value() {
                odometry.imu.update();
                return odometry.imu.getZAngle();
            }
        });


        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();
        odometry.start(rb.frMotor.getCurrentPosition(), rb.flMotor.getCurrentPosition(), rb.blMotor.getCurrentPosition());

        Thread.sleep(1500);
//        rb.goToTheta(Math.PI / 4, 0.2, odometry, runtime, 20);

        // Go forward to the foundation
        rb.goToX(87, 0.4, odometry);

        // Use the leadscrew to grab the foundation
        rb.runLeadscrewToPosition(-1700);

        // Pull the foundation back to the building site
//        rb.goToX(40, 0.7, odometry);

//        rb.goToTheta(3 * Math.PI / 16, 0.3, odometry, runtime, 4);
//
//        rb.goToX(5, 0.7, odometry);
//
//        rb.goToTheta(3 * Math.PI / 16, 0.3, odometry, runtime, 4);
//
//        rb.goToX(-40, 0.7, odometry);
//
//        rb.goToTheta(3 * Math.PI / 16, 0.3, odometry, runtime, 4);
//
//        rb.goToX(-75, 0.7, odometry);
//
//        rb.goToTheta(3 * Math.PI / 16, 0.3, odometry, runtime, 4);
//
//        rb.goToX(-100, 0.7, odometry);
//
//        rb.goToTheta(3 * Math.PI / 16, 0.2, odometry, runtime, 3);

        rb.goToX(-50, 0.7, odometry);


        // Let go of the foundation
        rb.runLeadscrewToPosition(-50);

        rb.goToTheta(Math.PI / 4, 0.3, odometry, runtime, 4);

        // Drive to the skybridge and park under it
        rb.goToY(-125, 0.4, odometry);
    }
}
