package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Claw;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleOpManual extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0, 0));

        //claw
        Claw claw = new Claw(hardwareMap);

        //lifter
        Lifter lifter = new Lifter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            mecanumDrive.update();
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double turn = gamepad1.right_stick_x;

            double fL = y+turn;
            double fR = y-turn;
            double bL = y+turn;
            double bR = y-turn;

            if(gamepad1.right_bumper) {
                mecanumDrive.setDrivePower(fL, fR, bL, bR);
            } else {
                mecanumDrive.setDrivePower(fL/2, fR/2, bL/2, bR/2);
            }

            if(!gamepad2.left_bumper && gamepad2.right_bumper) claw.open(); //open
            if(gamepad2.left_bumper && !gamepad2.right_bumper) claw.close(); //close

            lifter.move(-gamepad2.left_stick_y);

            telemetry.update();
        }
    }
}
