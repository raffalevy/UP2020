package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ServoTest extends OpMode {

    public static final double INCREMENT = .05;

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    OmniRobot rb = new OmniRobot();

    private Servo servo = null;

    public abstract Servo getTestServo();

    /**
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);
        servo = getTestServo();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * This method will be called once when the PLAY button is first pressed.
     */
    @Override
    public void start() {
        // Reset elapsed time
        runtime.reset();
    }

    public boolean leftWasDown = false;
    public boolean rightWasDown = false;
    public double servoPosition = .5;

    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");
        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;
                servoPosition -= INCREMENT;
            }
        } else {
            leftWasDown = false;
        }

        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
                servoPosition += INCREMENT;

            }
        } else {
            rightWasDown = false;
        }

        if (gamepad1.a){
          servo.setPosition(servoPosition);
        }

        telemetry.addData("ServoPosition", servoPosition);
        telemetry.update();

    }

}