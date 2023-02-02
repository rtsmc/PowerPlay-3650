package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        //reverse two motors on one side so positive values always make wheels spin forwards
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //set motors to brake when stopping so the robot doesn't slide
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset the encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //drive by setting velocities for all motors
    public void driveVelocity(double powerfL, double powerfR, double powerbL, double powerbR){
        frontLeft.setVelocity(powerfL*2800);
        frontRight.setVelocity(powerfR*2800);
        backLeft.setVelocity(powerbL*2800);
        backRight.setVelocity(powerbR*2800);
    }

    public void drive(double powerfL, double powerfR, double powerbL, double powerbR){
        frontLeft.setPower(powerfL);
        frontRight.setPower(powerfR);
        backLeft.setPower(powerbL);
        backRight.setPower(powerbR);
    }

    //get velocities of motors, useful for debugging
    public String getVelocity(){
        return "FrontLeft " + frontLeft.getVelocity() + "\n FrontRight " + frontRight.getVelocity() + "\n BackLeft " + backLeft.getVelocity() + "\n backRight " + backRight.getVelocity();
    }

    public int getPosition(){
        return frontLeft.getCurrentPosition();
    }
}
