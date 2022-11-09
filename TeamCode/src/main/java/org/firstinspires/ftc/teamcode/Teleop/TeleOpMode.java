package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightRear"), true);
        Claw claw = new Claw(hardwareMap.get(CRServo.class, "clawServo"));

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        mecanumDrive.setModes();

        waitForStart();

        while(opModeIsActive()){
            //Get the hypotenuse of the right triangle created by the position of the left gamepad stick on the x and y axis
            double gamepadX = -gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            double r = Math.hypot(gamepadX, gamepadY);

            /*Get the angle that the left gamepad stick creates with the horizontal x axis.
            This angle is then offset by PI/4 so that when the stick is
            all the way in one direction the sin and cos are the same.*/
            double gyroAngle = imu.getAngularOrientation().firstAngle;
            double cosA = Math.cos(gyroAngle);
            double sinA = Math.sin(gyroAngle);
            double x = gamepadX*cosA - gamepadY*sinA;
            double y = gamepadX*sinA + gamepadY*cosA;
            double angle = Math.atan2(y, x) - Math.PI/4;

            /*Set the variables for the power of each of the motors to the inverse of either the cos or sin of the angle above,
            then multiply by the hypotenuse to get speed.*/
            double fL = Math.sin(angle)*r;
            double fR = Math.cos(angle)*r;
            double bL = Math.cos(angle)*r;
            double bR = Math.sin(angle)*r;

            //Add or subract the x value of the right stick to each of the motors so that the
            //robot can turn with the right stick.
            fL += gamepad1.right_stick_x;
            fR -= gamepad1.right_stick_x;
            bL += gamepad1.right_stick_x;
            bR -= gamepad1.right_stick_x;

            mecanumDrive.drive(fL, fR, bL, bR);

            if (leftBumper) claw.move(1); //open
            else if (rightBumper) claw.move(-1); //close
            else claw.move(0);


            telemetry.update();
        }
    }
}