package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IMUTest", group = "EncoderTest")
public class IMUTest extends OpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    OmniRobot rb = new OmniRobot();

    IMU imu = new IMU();

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

        imu.initIMU(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    float startingAngle = 0;

    @Override
    public void start() {
        runtime.reset();
        imu.update();
        startingAngle = imu.getZAngle();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        imu.update();

        telemetry.addLine("Orientation:");
        telemetry.addData("ZA", imu.getZAngle() - startingAngle);
        telemetry.addData("Z", imu.getZAngle());
        telemetry.addData("Y", imu.getYAngle());
        telemetry.addData("X", imu.getXAngle());

        telemetry.update();

    }

}
