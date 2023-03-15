package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final CRServo servo;
    private final double power = 0.25;
    private long setTime;
    private boolean isBusy = false;
    public enum State {
        OPEN,
        CLOSED
    }
    private State state = State.OPEN;
    public Claw (HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "clawServo");
    }

    public void open() {
        if(state != State.OPEN) {
            setTime = System.currentTimeMillis();
            isBusy = true;
            servo.setPower(-power);
            state = State.OPEN;
        }
    }

    public void close() {
        if(state != State.CLOSED) {
            setTime = System.currentTimeMillis();
            isBusy = true;
            servo.setPower(power);
            state = State.CLOSED;
        }
    }

    public void update() {
        if(System.currentTimeMillis() - setTime >= 275) {
            if(state == State.OPEN){
                servo.setPower(0);
            }
            isBusy = false;
        }
    }

    public void setPower(double cpower){
        servo.setPower(cpower);
    }

    public boolean isBusy() {
        return isBusy;
    }

    public String getState() {
        return state.toString();
    }
}
