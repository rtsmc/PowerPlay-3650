package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Claw;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        //claw
        Claw claw = new Claw(hardwareMap);

        //imu
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        //lifter
        Lifter lifter = new Lifter(hardwareMap);
        boolean openLast = false;
        boolean dPadPressed = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            mecanumDrive.update();
            double gamepadX = -gamepad1.left_stick_x*1.1;
            double gamepadY = -gamepad1.left_stick_y;
            double r = Math.hypot(gamepadX, gamepadY);

            double gyroAngle = imu.getAngularOrientation().firstAngle-Math.PI/2;

            double cosA = Math.cos(gyroAngle);
            double sinA = Math.sin(gyroAngle);
            double x = gamepadX*cosA - gamepadY*sinA;
            double y = gamepadX*sinA + gamepadY*cosA;

            double angle = Math.atan2(y, x) - Math.PI/4;

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
                mecanumDrive.setDrivePower(fL, fR, bL, bR);
            } else {
                mecanumDrive.setDrivePower(fL/2, fR/2, bL/2, bR/2);
            }

            if(!gamepad2.left_bumper && gamepad2.right_bumper) {
                claw.setPower(-0.3); //open
                openLast = true;
            }
            if(gamepad2.left_bumper && !gamepad2.right_bumper) {
                claw.setPower(0.3); //close
                openLast = false;
            }
            if(gamepad2.left_bumper == gamepad2.right_bumper) {
                if(openLast) {
                    claw.setPower(0.01);
                } else {
                    claw.setPower(-0.01);
                }
            }

            if (!dPadPressed && gamepad2.dpad_up && !gamepad2.dpad_down) lifter.changeTarget(225); //up one cone
            if (!dPadPressed && !gamepad2.dpad_up && gamepad2.dpad_down) lifter.changeTarget(-225); //down one cone
            dPadPressed = gamepad2.dpad_up || gamepad2.dpad_down;

            if (gamepad2.a) lifter.setTargetPosition(0);
            if (gamepad2.b) lifter.setTargetPosition(1);
            if (gamepad2.x) lifter.setTargetPosition(2);
            if (gamepad2.y) lifter.setTargetPosition(3);

            lifter.changeTarget((int)(-50*gamepad2.left_stick_y));
            lifter.runToTarget();

            telemetry.addData("gyroAngle", gyroAngle);
            telemetry.addData("lifterTarget", lifter.getTargetPosition());
            telemetry.addData("lifterPosition", lifter.getPositions()[0]);
            telemetry.update();
        }
    }
}