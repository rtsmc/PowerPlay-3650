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
            double gamepadX = -gamepad1.left_stick_x*1.1;
            double gamepadY = -gamepad1.left_stick_y;
            double r = Math.hypot(gamepadX, gamepadY);
            //get angle from imu
            double gyroAngle = imu.getAngularOrientation().firstAngle;
            //rotate vector created by components gamepadX and gamepadY counter clockwise by the IMU angle, for field-centric drive
            double cosA = Math.cos(gyroAngle);
            double sinA = Math.sin(gyroAngle);
            double x = gamepadX*cosA - gamepadY*sinA;
            double y = gamepadX*sinA + gamepadY*cosA;

            //get angle of the rotated vector and offset by 45˚ or PI/4, since mecanum
            //wheels have rollers at 45˚ to the forward direction, rotating by this amount
            //makes the force vectors of the mecanum wheels match up with x and y components of
            //the joystick position
            double angle = Math.atan2(y, x) - Math.PI/4;

            //Set powers for each wheel of the robot.
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double turn = gamepad1.right_stick_x;

            double fL = ((sin*r)/max)+turn;
            double fR = ((cos*r)/max)-turn;
            double bL = ((cos*r)/max)+turn;
            double bR = ((sin*r)/max)-turn;

            if((r + Math.abs(turn)) > 1){
                fL /= (r + Math.abs(turn));
                fR /= (r + Math.abs(turn));
                bL /= (r + Math.abs(turn));
                bR /= (r + Math.abs(turn));
            }

            fL /= 1.1;
            bL /= 1.1;

            if(gamepad1.right_bumper) {
                mecanumDrive.drive(fL, fR, bL, bR);
            } else {
                mecanumDrive.drive(fL/2, fR/2, bL/2, bR/2);
            }

            if(!gamepad2.left_bumper && gamepad2.right_bumper) claw.open(); //open
            if(gamepad2.left_bumper && !gamepad2.right_bumper) claw.close(); //close
            if(gamepad2.left_bumper == gamepad2.right_bumper) claw.stop(); //stop

            if (gamepad2.dpad_up && !gamepad2.dpad_down) lifter.moveUp(); //up
            if (!gamepad2.dpad_up && gamepad2.dpad_down) lifter.moveDown(); //down
            if (gamepad2.dpad_up == gamepad2.dpad_down) lifter.stop(); //stop

            if (gamepad2.a) lifter.setTargetPosition(0);
            if (gamepad2.b) lifter.setTargetPosition(1);
            if (gamepad2.x) lifter.setTargetPosition(2);
            if (gamepad2.y) lifter.setTargetPosition(3);

            lifter.changeTarget((int)(-50*gamepad2.left_stick_y));
            lifter.runToTarget();

            telemetry.addData("fL, fR, bL, bR", fL + " " + fR + " " + bL + " " + bR);
            telemetry.addData("Encoder position", mecanumDrive.getPosition());
            telemetry.update();
        }
    }
}