package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    private Servo servo;
    private final double openPosition = 0.74;
    private final double closedPosition = 0;

    public Claw (HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawServo");
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public double getPosition() {
        return servo.getPosition();
    }
    public void open() {
        servo.setPosition(openPosition);
    }

    public void close() {
        servo.setPosition(closedPosition);
    }
}
