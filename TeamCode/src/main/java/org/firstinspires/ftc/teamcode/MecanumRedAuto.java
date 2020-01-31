package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.HOOK1_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.HOOK1_UP;
import static org.firstinspires.ftc.teamcode.Constants.blConstant1;
import static org.firstinspires.ftc.teamcode.Constants.flConstant1;
import static org.firstinspires.ftc.teamcode.Constants.flConstant2;
import static org.firstinspires.ftc.teamcode.Constants.flConstant3;
import static org.firstinspires.ftc.teamcode.Constants.frConstant1;
import static org.firstinspires.ftc.teamcode.Constants.frConstant2;

@Autonomous(name = "MRedFAuto", group = "RedAuto")
public class MecanumRedAuto extends LinearOpMode {

    MecanumRobot rb = new MecanumRobot();
    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

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


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();

        Thread.sleep(1500);

        // Go backward to the foundation
        while (rb.flMotor.getCurrentPosition() < flConstant1 && rb.frMotor.getCurrentPosition() < frConstant1) {
            rb.driveForward(-.5);
        }

        // Use the hook to grab the foundation
        rb.hookServo1.setPosition(HOOK1_DOWN);

        // Pull the foundation back to the building site
        while (rb.flMotor.getCurrentPosition() > flConstant2 && rb.frMotor.getCurrentPosition() > frConstant2) {
            rb.driveForward(.7);
        }

        // Let go of the foundation
        rb.hookServo1.setPosition(HOOK1_UP);

        // Drive to the skybridge and park under it
        while (rb.flMotor.getCurrentPosition() > flConstant3 && rb.blMotor.getCurrentPosition() < blConstant1) {
            rb.strafeRight(5);
        }

    }
}
