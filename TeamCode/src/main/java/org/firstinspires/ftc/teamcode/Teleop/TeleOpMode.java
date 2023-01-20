package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightRear"));

        //claw
        Claw claw = new Claw(hardwareMap.get(CRServo.class, "clawServo"));

        //lifter
        Lifter lifter = new Lifter(hardwareMap.get(DcMotorEx.class, "leftLifter"),
                hardwareMap.get(DcMotorEx.class, "rightLifter"));

        //imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            //Get the hypotenuse of the right triangle created by the position of the left gamepad stick on the x and y axis
            double gamepadX = -gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;
            double r = Math.hypot(gamepadX, gamepadY);
            //get angle from imu
            double gyroAngle = imu.getAngularOrientation().firstAngle;
            //rotate vector created by components gamepadX and gamepadY counter clockwise by the IMU angle, for field-centric drive
            double cosA = Math.cos(gyroAngle);
            double sinA = Math.sin(gyroAngle);
            double x = gamepadX*cosA - gamepadY*sinA;
            double y = gamepadX*sinA + gamepadY*cosA;
            //get angle of the rotated vector and offset by PI/4, so that
            //straight up is PI/4 instead of PI/2, so all wheels spin
            //the same velocity & robot moves forward when joystick is pushed straight up.
            double angle = Math.atan2(y, x) - Math.PI/4;
            //Set powers for each wheel of the robot.
            double fL = Math.sin(angle)*r;
            double fR = Math.cos(angle)*r;
            double bL = Math.cos(angle)*r;
            double bR = Math.sin(angle)*r;
            //Add or subract the x value of the right stick from motor powers to give turning control.
            fL += gamepad1.right_stick_x;
            fR -= gamepad1.right_stick_x;
            bL += gamepad1.right_stick_x;
            bR -= gamepad1.right_stick_x;

            if(gamepad1.right_bumper){
                mecanumDrive.driveVelocity(fL, fR, bL, bR);
            } else {
                mecanumDrive.driveVelocity(fL/2, fR/2, bL/2, bR/2);
            }

            if(!gamepad2.left_bumper && gamepad2.right_bumper) {
                claw.open(); //open
            }
            if(gamepad2.left_bumper && !gamepad2.right_bumper) {
                claw.close(); //close
            }
            if(gamepad2.left_bumper == gamepad2.right_bumper) {
                claw.stop(); //stop
            }

            if (gamepad2.dpad_up && !gamepad2.dpad_down) lifter.moveUp(); //up
            if (!gamepad2.dpad_up && gamepad2.dpad_down) lifter.moveDown(); //down
            if (gamepad2.dpad_up == gamepad2.dpad_down) lifter.stop(); //stop

            if (gamepad2.a) lifter.setTargetPosition(0);
            if (gamepad2.b) lifter.setTargetPosition(1);
            if (gamepad2.x) lifter.setTargetPosition(2);
            if (gamepad2.y) lifter.setTargetPosition(3);

            lifter.changeTarget((int)(-50*gamepad2.left_stick_y));
            lifter.runToTarget();

            telemetry.addData("Lifter positions", "Left = "+lifter.getPositions()[0]+", Right = "+lifter.getPositions()[1]);
            telemetry.addData("Target position", lifter.getTargetPosition());
            telemetry.addData("Current mode", lifter.getMode());
            telemetry.addData("gamepad2 left bumper", gamepad2.left_bumper);
            telemetry.addData("gamepad2 right bumper", gamepad2.right_bumper);
            telemetry.update();
        }
    }
}