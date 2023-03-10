package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    private Servo servo;
    // Note: Higher number = more closed

    private final double openPosition = 0.275; // maximum open: 0.25
    private final double closedPosition = 1; // minimum close: 0.4

    public Claw (HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawServo");
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
