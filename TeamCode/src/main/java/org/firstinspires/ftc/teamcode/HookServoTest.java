package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HookServoTest", group = "MecanumServoTest")
public class HookServoTest extends MecanumServoTest {
    @Override
    public Servo getTestServo() {
        return rb.hookServo;
    }
}
