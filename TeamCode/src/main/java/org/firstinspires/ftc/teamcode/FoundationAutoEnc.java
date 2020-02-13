package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FoundationAutoEnc", group = "FoundationAuto")
public class FoundationAutoEnc extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    MecanumRobot rb = new MecanumRobot();

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

        rb.hooksUp();

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();

        rb.strafeRightByEncoder(374, rb.flMotor, .4);
        rb.driveForwardByEncoder(-1330, rb.flMotor, .4);
        rb.hooksDown();
        Thread.sleep(1250);
        rb.driveForwardByEncoder(1330, rb.flMotor, .4);
        rb.hooksUp();
        rb.strafeRightByEncoder(-1500, rb.flMotor, .4);

//        while (runtime.seconds()<2){
//            rb.driveForward(-.5);
//            telemetry.addData("Fr",rb.frMotor.getCurrentPosition());
//            telemetry.addData("Fl",rb.flMotor.getCurrentPosition());
//            telemetry.addData("Br",rb.brMotor.getCurrentPosition());
//            telemetry.update();
//        }
//            rb.hooksDown();
//        Thread.sleep(500);
//        while (runtime.seconds()<10){
//            rb.driveForward(.7);
//            telemetry.addData("Fr",rb.frMotor.getCurrentPosition());
//            telemetry.addData("Fl",rb.flMotor.getCurrentPosition());
//            telemetry.addData("Br",rb.brMotor.getCurrentPosition());
//            telemetry.update();
//        }
//            rb.hooksUp();
//        while (runtime.seconds()<15){
//            rb.strafeRight(.5);
//            telemetry.addData("Fr",rb.frMotor.getCurrentPosition());
//            telemetry.addData("Fl",rb.flMotor.getCurrentPosition());
//            telemetry.addData("Br",rb.brMotor.getCurrentPosition());
//            telemetry.update();
//        }
    }
}
