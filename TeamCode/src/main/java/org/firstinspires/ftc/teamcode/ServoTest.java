package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD_SQUARED;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "UPTeleOp", group = "TeleOp")
public class ServoTest extends OpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    private OmniRobot rb = new OmniRobot();

    private Servo servo = null;

    /**
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap);

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
    private double increment = .05;

    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        if (gamepad1.left_bumper) {
            if (!leftWasDown) {
                leftWasDown = true;

            }
        } else {
            leftWasDown = false;
        }

        if (gamepad1.right_bumper) {
            if (!rightWasDown) {
                rightWasDown = true;
            }
        } else {
            rightWasDown = false;
        }
        if (leftWasDown){
            servoPosition -= increment;
        }
        if (rightWasDown){
            servoPosition += increment;
        }
        if (gamepad1.a){
          servo.setPosition(servoPosition);
        }
        telemetry.addData("ServoPosition", servoPosition);
        telemetry.update();

    }

}