package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lifter {
    private DcMotorEx left;
    private DcMotorEx right;

    public Lifter(DcMotorEx left, DcMotorEx right){
        this.left = left;
        this.right = right;

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveUp(){
        left.setVelocity(900);
        right.setVelocity(900);
    }

    public void moveDown(){
        left.setVelocity(-900);
        right.setVelocity(-900);
    }

    public void stop(){
        left.setVelocity(0);
        right.setVelocity(0);
    }
}