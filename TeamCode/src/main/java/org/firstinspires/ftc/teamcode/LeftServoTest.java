package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LeftServoTest", group = "ServoTest")
public class LeftServoTest extends ServoTest {
    @Override
    public Servo getTestServo() {
        return rb.leftServo;
    }
}
