package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RightServoTest", group = "ServoTest")
public class RightServoTest extends ServoTest {
    @Override
    public Servo getTestServo() {
        return rb.rightServo;
    }
}
