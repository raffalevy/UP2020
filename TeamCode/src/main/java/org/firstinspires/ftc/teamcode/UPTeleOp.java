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
        rb.leftServo.setPosition(LS_BACK);
        rb.rightServo.setPosition(RS_BACK);

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

        moveGrabbers();

        moveLift();
    }

    /**
     * Drive the chassis according to joysticks
     */
    private void driveChassis() {
        float leftY = -gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x;
        float rightX = gamepad1.right_stick_x;

        double pow;
        if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            pow = DRIVE_POWER_SLOW;
        } else {
            pow = DRIVE_POWER;
        }

        if (leftX * leftX + leftY * leftY >= DRIVE_STICK_THRESHOLD_SQUARED || Math.abs(rightX) >= DRIVE_STICK_THRESHOLD) {
            rb.drive(leftX, leftY, rightX, pow);
        } else {
            rb.driveStop();
        }
    }

    private void moveGrabbers() {
        if (gamepad2.left_trigger >= TRIGGER_THRESHOLD) {
            rb.leftServo.setPosition(LS_BACK);
            rb.rightServo.setPosition(RS_BACK);
        } else if (gamepad2.right_trigger >= TRIGGER_THRESHOLD) {
            rb.leftServo.setPosition(LS_PARALLEL);
            rb.rightServo.setPosition(RS_PARALLEL);
        } else if (gamepad2.left_bumper) {
            rb.leftServo.setPosition(LS_PERP);
            rb.rightServo.setPosition(RS_PERP);
        } else if (gamepad2.right_bumper) {
            rb.leftServo.setPosition(LS_GRAB);
            rb.rightServo.setPosition(RS_GRAB);
        }
    }

    private void moveLift() {
        if (Math.abs(gamepad2.left_stick_y) >= 0.5) {
            rb.liftMotor.setPower(gamepad2.left_stick_y / 2);
        } else {
            rb.liftMotor.setPower(0);
        }
    }
}