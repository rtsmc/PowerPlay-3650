package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Auto extends LinearOpMode{

    //this enum defines the state of the robot
    //use this for the switch statement to asynchronously control the robot
    //to use lifer and move at the same time

    enum State {
        Start_Pole,        // Start -> first pole (raise arm)
        Park,              // parks the robot
        IDLE               // Our bot will enter the IDLE state when done
    }

    OpenCvWebcam webcam;
    private ColorDetector detector;
    int[] detections = new int[100];
    private int level;

    State currentState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        //lifter
        Lifter lifter = new Lifter(hardwareMap.get(DcMotorEx.class, "leftLifter"),
                hardwareMap.get(DcMotorEx.class, "rightLifter"));

        //drive
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

        //Claw
        Claw claw = new Claw(hardwareMap.get(CRServo.class, "clawServo"));

        //Set intial postion
        Pose2d startPose = new Pose2d(36, 65, Math.toRadians(270));  //change this based on the starting location
        mecanumDrive.setPoseEstimate(startPose);

        //opencv
        detector = new ColorDetector();
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

        //trajectories

        //Starting location to first pole
        Trajectory startPole = mecanumDrive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(34, 20, Math.toRadians(230)), Math.toRadians(250))
                .splineToConstantHeading(
                        new Vector2d(31, 6.5), Math.toRadians(250),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //pole to parking 1
        Trajectory poleParking1 = mecanumDrive.trajectoryBuilder(startPole.end(), true)
            .splineToSplineHeading(new Pose2d(36.0, 28, Math.toRadians(270.0)), Math.toRadians(100.0))
            .splineToConstantHeading(new Vector2d(63.0, 36.0), 0.0)
            .build();

        //pole to parking 2
        Trajectory poleParking2 = mecanumDrive.trajectoryBuilder(startPole.end(), true)
            .splineToSplineHeading(new Pose2d(36.0, 36.0, Math.toRadians(270.0)), Math.toRadians(100.0))
            .build();

        //pole to parking 3
        Trajectory poleParking3 = mecanumDrive.trajectoryBuilder(startPole.end(), true)
            .splineToConstantHeading(new Vector2d(36, 24), Math.toRadians(230))
            .splineToConstantHeading(new Vector2d(60, 36), Math.toRadians(230))
            .build();
        waitForStart();

        currentState = State.Start_Pole;
        claw.close();
        mecanumDrive.followTrajectoryAsync(startPole);
        lifter.setTargetPosition(3);

        while(opModeIsActive()){
            telemetry.addData("level", level);
            telemetry.update();
            switch (currentState) {
                case Start_Pole:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.Park;
                        claw.open();
                        sleep(400);
                        lifter.setTargetPosition(0);
                        if (level == 1) {
                            mecanumDrive.followTrajectoryAsync(poleParking1);
                        } else if (level == 2) {
                            mecanumDrive.followTrajectoryAsync(poleParking2);
                        } else if (level == 3){
                            mecanumDrive.followTrajectoryAsync(poleParking3);
                        }
                    }
                    break;
                case Park:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            mecanumDrive.update();
            lifter.runToTarget();
        }
    }

    private int roundedAverage(int[] arr) {
        double sum = 0;
        for(int i = 0; i < arr.length; i++){
            sum+=arr[i];
        }
        return (int)(Math.round(sum/arr.length));
    }
}