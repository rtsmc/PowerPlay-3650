package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lifter {
    private DcMotorEx left;
    private DcMotorEx right;
    private static final int LIFTER_VELOCITY = 2700;
    private static final int[] POSITIONS = {0, 1800, 3733, 5600};

    public Lifter(DcMotorEx left, DcMotorEx right){
        this.left = left;
        this.right = right;

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveUp(){
        left.setVelocity(LIFTER_VELOCITY);
        right.setVelocity(LIFTER_VELOCITY);
    }

    public void moveDown(){
        left.setVelocity(-LIFTER_VELOCITY);
        right.setVelocity(-LIFTER_VELOCITY);
    }

    public void stop(){
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public void setPosition(int position){
        left.setTargetPosition(POSITIONS[position]);
        right.setTargetPosition(POSITIONS[position]);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public String getPositions(){
        return "left: " + left.getCurrentPosition() + " right: " + right.getCurrentPosition();
    }
}