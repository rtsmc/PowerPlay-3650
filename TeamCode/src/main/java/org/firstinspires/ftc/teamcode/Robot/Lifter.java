package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lifter {
    private DcMotorEx left;
    private DcMotorEx right;
    private static final int LIFTER_VELOCITY = 2700;
    private static final int[] POSITIONS = {0, 2600, 4200, 5800, 850, 650, 450, 250, 50};
    private static int targetPosition;
    private static int mode; // 0: Manual; 1: Run to position

    public Lifter(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotorEx.class, "leftLifter");
        right = hardwareMap.get(DcMotorEx.class, "rightLifter");
        targetPosition = 0;
        mode = 0;

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUp() {
        mode = 0; // Set mode to Manual
        left.setVelocity(LIFTER_VELOCITY);
        right.setVelocity(LIFTER_VELOCITY);
    }

    public void moveDown() {
        mode = 0; // Set mode to Manual
        left.setVelocity(-LIFTER_VELOCITY);
        right.setVelocity(-LIFTER_VELOCITY);
    }

    public void stop() {
        if (mode != 0) return; // Only in Manual mode
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public void setTarget(int target) {
        if (target < 0) target = 0;
        targetPosition = target;
    }

    public void setTargetPosition(int position) {
        mode = 1;
        setTarget(POSITIONS[position]);
    }

    public void changeTarget(int amount) {
        if (amount != 0) mode = 1; // Set mode to Run to position
        setTarget(targetPosition+amount);
    }

    public void runToTarget() {
        if (mode != 1) return; // Only in Run to position mode

        // Calculate left velocity
        int leftVelocity = 3*(targetPosition-left.getCurrentPosition());
        if (leftVelocity < -LIFTER_VELOCITY) leftVelocity = -LIFTER_VELOCITY;
        if (leftVelocity > LIFTER_VELOCITY) leftVelocity = LIFTER_VELOCITY;
        if (leftVelocity < 30 && leftVelocity > -30) leftVelocity = 0;
        left.setVelocity(leftVelocity);

        //Calculate right velocity
        int rightVelocity = 3*(targetPosition-right.getCurrentPosition());
        if (rightVelocity < -LIFTER_VELOCITY) rightVelocity = -LIFTER_VELOCITY;
        if (rightVelocity > LIFTER_VELOCITY) rightVelocity = LIFTER_VELOCITY;
        if (rightVelocity < 30 && rightVelocity > -30) rightVelocity = 0;
        right.setVelocity(rightVelocity);
    }

    public int[] getPositions(){
        return new int[]{left.getCurrentPosition(), right.getCurrentPosition()};
    }

    public int getTargetPosition(){
        return targetPosition;
    }

    public int getMode() {
        return mode;
    }
}
