package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "EncoderOdometryTest", group = "Test")
public class EncoderOdometryTest extends LinearOpMode {
    private MecanumRobot rb = new MecanumRobot();

    private MecanumOdometry odometry = new MecanumOdometry();

    @Override
    public void runOpMode() throws InterruptedException {
        rb.init(hardwareMap,this);
        odometry.init(hardwareMap);
        rb.resetEncoder(rb.blMotor);
        rb.resetEncoder(rb.flMotor);
        rb.resetEncoder(rb.frMotor);
        telemetry.addData("FR Encoder",rb.frMotor.getCurrentPosition());
        telemetry.addData("FL Encoder",rb.flMotor.getCurrentPosition());
        telemetry.addData("BL Encoder",rb.blMotor.getCurrentPosition());
        telemetry.update();
        odometry.start(rb.frMotor.getCurrentPosition(),rb.flMotor.getCurrentPosition(),rb.blMotor.getCurrentPosition());
        while (opModeIsActive()){
            odometry.update(rb.frMotor.getCurrentPosition(),rb.flMotor.getCurrentPosition(),rb.blMotor.getCurrentPosition());
            telemetry.addData("FR Encoder",rb.frMotor.getCurrentPosition());
            telemetry.addData("FL Encoder",rb.flMotor.getCurrentPosition());
            telemetry.addData("BL Encoder",rb.blMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
