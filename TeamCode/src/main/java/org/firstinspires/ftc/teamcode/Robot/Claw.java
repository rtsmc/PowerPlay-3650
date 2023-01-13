package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;

public class Claw {
    private CRServo servo;
    private int power = 1;
    private boolean closed = false;
    private int movementTime = 250;
    private long startTime;

    public Claw (CRServo servo) {
        this.servo = servo;
    }

    public void open() {
        startTime = System.currentTimeMillis();
        servo.setPower(-power);
    }

    public void close() {
        startTime = System.currentTimeMillis();
        servo.setPower(power);
    }

    public void stop() { servo.setPower(0); }

    public void update() {
        if(System.currentTimeMillis() - startTime >= movementTime){
            stop();
        }
    }

}
