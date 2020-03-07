package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD_SQUARED;
import static org.firstinspires.ftc.teamcode.Constants.LS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.LS_UP;
import static org.firstinspires.ftc.teamcode.Constants.RS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.RS_UP;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

/**
 * TeleOp for the new Mecanum robot
 *
 * December 2019
 */
@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();
    /**
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        telemetry.addData("Status", "Initialized");
    }
    /**
     * This method will be called once when the PLAY button is first pressed.
     */
    @Override
    public void start() {
        // Reset elapsed time
        runtime.reset();
        rb.resetEncoder(rb.frMotor);
        rb.resetEncoder(rb.flMotor);
        rb.resetEncoder(rb.blMotor);
        rb.resetEncoder(rb.brMotor);
    }
    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        driveChassis();

        moveGrabbers();

        moveHooks();

        telemetry.addData("frEncoder", rb.frMotor.getCurrentPosition());
        telemetry.addData("flEncoder", rb.flMotor.getCurrentPosition());
        telemetry.addData("blEncoder", rb.blMotor.getCurrentPosition());
        telemetry.update();

    }

    private void moveGrabbers() {
        if (gamepad2.left_bumper) {
            rb.release();
        } else if (gamepad2.right_bumper) {
            rb.grab();
        }
    }

    private boolean aDown = false;
    private boolean hooksDown = false;
    private void moveHooks() {
        if (gamepad1.a) {
            if (!aDown) {
                aDown = true;
                hooksDown = !hooksDown;
            }
        } else {
            aDown = false;
        }

        if (hooksDown) {
            rb.hooksDown();
        } else {
            rb.hooksUp();
        }
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
}

