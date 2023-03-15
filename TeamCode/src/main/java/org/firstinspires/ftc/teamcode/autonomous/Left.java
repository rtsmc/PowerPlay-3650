package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Claw;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class Left extends LinearOpMode {
    enum State {
        Start_Pole,
        Park,
        Drop,
        IDLE
    }
    private OpenCvWebcam webcam;
    private final int[] detections = new int[100];
    private int level;
    @Override
    public void runOpMode() throws InterruptedException {
        State currentState;
        Lifter lifter = new Lifter(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(0));
        ColorDetector detector = new ColorDetector();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for Start");
        telemetry.update();
        int i = 0;
        while(!isStarted()){
            if(i == 99){
                i = 0;
            }
            detections[i] = detector.getLevel();
            i++;
            telemetry.addData("average of detections", roundedAverage(detections));
            level = roundedAverage(detections);
            telemetry.update();
        }

        Trajectory startPole = drive.trajectoryBuilder(startPose, 90)
                .splineToConstantHeading(new Vector2d(-34, -26), 80)
                .build();

        Trajectory poleParking1 = drive.trajectoryBuilder(startPole.end(), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-36, -32), 230)
                .splineToConstantHeading(new Vector2d(-60, -36), 180)
                .build();

        Trajectory poleParking3 = drive.trajectoryBuilder(startPole.end(), Math.toRadians(240))
                .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(0))
                .build();

        waitForStart();

        currentState = State.Start_Pole;
        claw.setPower(0.3);
        lifter.setTargetPosition(3);
        drive.followTrajectoryAsync(startPole);

        while(opModeIsActive()){
            switch(currentState){
                case Start_Pole:
                    if(!drive.isBusy()){
                        currentState = State.Drop;
                        lifter.setTarget(5400);
                    }
                    break;
                case Drop:
                    if(!lifter.isBusy()){
                        claw.setPower(-0.3);
                        sleep(270);
                        claw.setPower(0.01);
                        currentState = State.Park;
                        lifter.setTargetPosition(0);
                        if (level == 1) drive.followTrajectoryAsync(poleParking1);

                        if (level == 3) drive.followTrajectoryAsync(poleParking3);
                    }
                    break;
                case Park:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            lifter.runToTarget();
        }
    }

    private int roundedAverage(int[] arr) {
        double sum = 0;
        for (int i : arr) {
            sum += i;
        }
        return (int)(Math.round(sum/arr.length));
    }
}