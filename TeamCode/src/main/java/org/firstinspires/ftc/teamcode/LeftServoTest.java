package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LeftServoTest", group = "MecanumServoTest")
public class LeftServoTest extends MecanumServoTest {
    @Override
    public Servo getTestServo() {
        return rb.leftServo;
    }
}
