

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

import java.io.*;
import java.lang.Thread;

@Autonomous
public class Auto extends LinearOpMode{

    //this enum defines the state of the robot
    //use this for the switch statement to asynchronously control the robot
    //to use lifer and move at the same time

    enum State {
        Start_Pole,        // Start -> first pole (raise arm)
        Pole_Cone,         // first pole -> cones (lower arm)
        Cone_Pole,         // cones -> pole (raise arm)
        Park,              // parks the robot
        /*
        Pole_Parking1,     // pole -> parking 1
        Pole_Parking2,     // pole -> parking 2
        Pole_Parking3,     // pole -> parking 3
         */
        Drop,              // Drop Cone
        Grab,              // Grab Cone
        IDLE               // Our bot will enter the IDLE state when done
    }

    OpenCvWebcam webcam;
    private ColorDetector detector;
    int[] detections = new int[100];
    private int level;

    State currentState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        //keep track of cycles between cones and pole
        int cycle = 0;
        
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

        //var to keep track of inital time for delays
        Thread t = new Thread("Sleep");
        long endTime;
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
            telemetry.update();
        }

        //trajectories

        //Starting location to first pole
        Trajectory startPole = mecanumDrive.trajectoryBuilder(startPose)
            .splineToSplineHeading(new Pose2d(34, 20, Math.toRadians(240)), Math.toRadians(250))
            .splineToConstantHeading(
                    new Vector2d(29, 6), Math.toRadians(250),
                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

        //Pole to cones
        Trajectory poleCone = mecanumDrive.trajectoryBuilder(startPole.end())
            .splineToSplineHeading(new Pose2d(29.001, 6, Math.toRadians(240)), Math.toRadians(60),
                    SampleMecanumDrive.getVelocityConstraint(1, 1, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .splineToSplineHeading(new Pose2d(58, 12, Math.toRadians(0)), Math.toRadians(0))
            .splineToConstantHeading(
                    new Vector2d(69, 12), Math.toRadians(0),
                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

        //cones to pole
        Trajectory conePole = mecanumDrive.trajectoryBuilder(poleCone.end())
            .splineToSplineHeading(new Pose2d(69.0001, 12, Math.toRadians(0)), Math.toRadians(180),
                    SampleMecanumDrive.getVelocityConstraint(1, 1, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .splineToSplineHeading(new Pose2d(37, 10, Math.toRadians(230)), Math.toRadians(220))
            .splineToConstantHeading(
                    new Vector2d(30, 5.5), Math.toRadians(200),
                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

        //pole to parking 1
        Trajectory poleParking1 = mecanumDrive.trajectoryBuilder(conePole.end())
            .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
            .splineToSplineHeading(new Pose2d(36, 24, Math.toRadians(300)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(12, 36, Math.toRadians(270)), Math.toRadians(0))
            .build();

        //pole to parking 2
        Trajectory poleParking2 = mecanumDrive.trajectoryBuilder(conePole.end())
            .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
            .splineToSplineHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(90))
            .build();

        //pole to parking 3
        Trajectory poleParking3 = mecanumDrive.trajectoryBuilder(conePole.end())
            .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
            .splineToSplineHeading(new Pose2d(36, 24, Math.toRadians(270)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(60, 36, Math.toRadians(270)), Math.toRadians(180))
            .build();


        waitForStart();

        currentState = State.Start_Pole;
        claw.close();
        lifter.setTargetPosition(3);
        mecanumDrive.followTrajectoryAsync(startPole);

        while(opModeIsActive()){
            level = roundedAverage(detections);
            telemetry.addData("level", level);
            telemetry.update();
            switch (currentState) {
                case Start_Pole:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.Pole_Cone;
                        claw.open();
                        lifter.setTargetPosition(4 + cycle);
                        mecanumDrive.followTrajectoryAsync(poleCone);
                    }
                    break;
                case Pole_Cone:
                    if (lifter.getPositions()[0] < 4000) {
                        claw.close();
                    }

                    if (lifter.getPositions()[0] < 1200) {
                        claw.open();
                    }

                    if (!mecanumDrive.isBusy()) {
                        claw.close();
                        t.start();
                        t.sleep(1000);
//                        endTime = System.currentTimeMillis() + 200;
//                        while (true) {
//                            if (System.currentTimeMillis() >= endTime) {
//                                break;
//                            }
//                        }
                        lifter.setTargetPosition(3);
                        currentState = State.Grab;

                    }
                    break;
                case Grab:
                    if(lifter.getPositions()[0] > 2000){
                        currentState = State.Cone_Pole;
                        mecanumDrive.followTrajectoryAsync(conePole);
                    }
                    break;
                case Cone_Pole:
                    if (!mecanumDrive.isBusy()) {
                        cycle++;
                        if (cycle == 5) {
                            lifter.setTargetPosition(0);
                            currentState = State.Park;
                            if (level == 1) mecanumDrive.followTrajectoryAsync(poleParking1);

                            if (level == 2) mecanumDrive.followTrajectoryAsync(poleParking2);

                            if (level == 3) mecanumDrive.followTrajectoryAsync(poleParking2);
                        }
                        else {
                            currentState = State.Pole_Cone;
                            lifter.setTargetPosition(4);
                            claw.open();
                            sleep(400);
                            mecanumDrive.followTrajectoryAsync(poleCone);
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