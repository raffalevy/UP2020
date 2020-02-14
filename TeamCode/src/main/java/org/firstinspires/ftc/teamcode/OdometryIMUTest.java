package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "OdometryIMUTest", group = "EncoderTest")
public class OdometryIMUTest extends OpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    OmniRobot rb = new OmniRobot();

    OdometryIMU odometry = new OdometryIMU();

    /**
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

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
    }

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * This method will be called once when the PLAY button is first pressed.
     */
    @Override
    public void start() {
        // Reset elapsed time
        runtime.reset();
        odometry.start(rb.frMotor.getCurrentPosition(),rb.flMotor.getCurrentPosition(),rb.blMotor.getCurrentPosition());
    }


    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        odometry.update(rb.frMotor.getCurrentPosition(),rb.flMotor.getCurrentPosition(),rb.blMotor.getCurrentPosition());

        telemetry.addData("x", odometry.getX());
        telemetry.addData("y", odometry.getY());
        telemetry.addData("theta", odometry.getTheta());

        telemetry.addData("FR", rb.frMotor.getCurrentPosition());
        telemetry.addData("FL", rb.flMotor.getCurrentPosition());
        telemetry.addData("BL", rb.blMotor.getCurrentPosition());
        telemetry.addData("BR", rb.brMotor.getCurrentPosition());

        telemetry.update();
    }
}