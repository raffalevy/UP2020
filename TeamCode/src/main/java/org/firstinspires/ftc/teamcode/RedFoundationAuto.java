package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

@Autonomous(name = "RedFAuto", group = "RedAuto")
public class RedFoundationAuto extends LinearOpMode {

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

        rb.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        waitForStart();

        runtime.reset();
        odometry.start(rb.frMotor.getCurrentPosition(), rb.flMotor.getCurrentPosition(), rb.blMotor.getCurrentPosition());

        rb.goToX(85, 0.4, odometry);

        rb.runLeadscrewToPosition(-1250);

        rb.goToX(0, 0.4, odometry);

        rb.runLeadscrewToPosition(-50);
    }
}
