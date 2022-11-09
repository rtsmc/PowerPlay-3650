package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;

public class Claw {
    private CRServo servo;
    public Claw(CRServo servo) {
        this.servo = servo;
    }
    public void move(int direction) {
        servo.setPower(direction);
    }
}
