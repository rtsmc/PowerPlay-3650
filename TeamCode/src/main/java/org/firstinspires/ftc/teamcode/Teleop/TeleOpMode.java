package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    private SampleMecanumDrive mecanumDrive;
    private Claw claw;
    private Lifter lifter;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        //claw
        claw = new Claw(hardwareMap);

        //lifter
        lifter = new Lifter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            double gamepadX = -gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;
            double r = Math.hypot(gamepadX, gamepadY);

            double gyroAngle = mecanumDrive.getRawExternalHeading();

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

            if(gamepad1.right_bumper) {
                mecanumDrive.setDrivePower(fL, fR, bL, bR);
            } else {
                mecanumDrive.setDrivePower(fL/2, fR/2, bL/2, bR/2);
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
        }
    }
}
