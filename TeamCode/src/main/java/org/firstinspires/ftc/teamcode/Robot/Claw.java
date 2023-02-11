package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    private CRServo servo;
    private final double power = 1;

    public Claw (HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "clawServo");
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
