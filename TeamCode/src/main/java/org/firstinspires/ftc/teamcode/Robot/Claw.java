package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;

public class Claw {
    private CRServo servo;
    private int power = 1;
    private boolean closed = false;

    public Claw (CRServo servo) {
        this.servo = servo;
    }

    public void open() {
        servo.setPower(-power);
    }

    public void close() {
        servo.setPower(power);
    }

    public void stop() { servo.setPower(0); }


    //auto claw code


    public void stayOpen() {
        closed = false;
    }

    public void stayClosed() {
        closed = true;
        while (closed) {
            servo.setPower(power);
            if (!closed) break;
        }

    }
}
