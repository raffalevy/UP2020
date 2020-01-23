package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RightServoTest", group = "MecanumServoTest")
public class RightServoTest extends MecanumServoTest {
    @Override
    public Servo getTestServo() {
        return rb.rightServo;
    }
}
