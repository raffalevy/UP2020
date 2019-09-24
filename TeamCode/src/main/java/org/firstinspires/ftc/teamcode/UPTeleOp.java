package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name = "UPTeleOp", group = "TeleOp")
public class UPTeleOp extends OpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    private OmniRobot rb = new OmniRobot();

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

    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        // Only use the sticks to drive if the dpad is not being pressed
        driveChassis();
    }

    /**
     * Drive the chassis according to joysticks
     */
    private void driveChassis() {
        float leftY = -gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x;
        float rightX = gamepad1.right_stick_x;

        float leftMagSquared = leftX * leftX + leftY * leftY;

        double forwardPower;
        double rightPower;

        double pow;
        if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            pow = DRIVE_POWER_SLOW;
        } else {
            pow = DRIVE_POWER;
        }

        // Calculate strafe from left stick
        if (leftMagSquared >= DRIVE_LEFT_STICK_THRESHOLD_SQUARED) {
            if (leftY > leftX) {
                if (leftY > -leftX) {
                    // Go forward
                    forwardPower = 1;
                    rightPower = 0;
                } else {
                    // Go left
                    forwardPower = 0;
                    rightPower = -1;
                }
            } else {
                if (leftY > -leftX) {
                    // Go right
                    forwardPower = 0;
                    rightPower = 1;
                } else {
                    // Go backwards
                    forwardPower = -1;
                    rightPower = 0;
                }
            }
        } else {
            forwardPower = 0;
            rightPower = 0;
        }

        double clockwisePower;

        if (Math.abs(rightX) > DRIVE_STICK_THRESHOLD) {
            clockwisePower = rightX;
        } else {
            clockwisePower = 0;
        }

        forwardPower *= pow;
        rightPower *= pow;
        clockwisePower *= pow;

        rb.drive(forwardPower, rightPower, clockwisePower);
    }
}