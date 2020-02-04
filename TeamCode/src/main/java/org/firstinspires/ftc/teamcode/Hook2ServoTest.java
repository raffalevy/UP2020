package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hook2ServoTest", group = "MecanumServoTest")
public class Hook2ServoTest extends MecanumServoTest {
    @Override
    public Servo getTestServo() {
        return rb.hookServo2;
    }
}
