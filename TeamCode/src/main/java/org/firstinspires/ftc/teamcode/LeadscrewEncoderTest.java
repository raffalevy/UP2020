package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LeadscrewEncoderTest", group = "EncoderTest")
public class LeadscrewEncoderTest extends OpMode {
//-1250
    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    OmniRobot rb = new OmniRobot();

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
        rb.leadscrewMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoder(rb.leadscrewMotor);

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
    }


    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        if (gamepad1.b) {
            resetEncoder(rb.leadscrewMotor);
        }

        telemetry.addData("LSc Position", rb.leadscrewMotor.getCurrentPosition());

        telemetry.update();
    }

}