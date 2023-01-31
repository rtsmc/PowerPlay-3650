package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;

public class Claw {
    private CRServo servo;
    long time;
    private final double power = 1;

    public Claw (CRServo servo) {
        this.servo = servo;
    }

    public void open() {
        servo.setPower(-power);
    }

    public void close() {
        servo.setPower(power);
    }

    public void stop() {
        servo.setPower(0);
    }
}
