/*
https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AsyncFollowingFSM.java

 */
package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
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


        //lifter
        Lifter lifter = new Lifter(hardwareMap.get(DcMotorEx.class, "leftLifter"),
                hardwareMap.get(DcMotorEx.class, "rightLifter"));

        //drive
                SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);


        //Claw
        Claw claw = new Claw(hardwareMap.get(CRServo.class, "clawServo"));

        //Set intial postion
        Pose2d startPose = new Pose2d(-36, 65, Math.toRadians(270));  //change this based on the starting location
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
            telemetry.update();
        }

        //trajectorys

            //Starting location to first pole
        Trajectory startPole = mecanumDrive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0.001, () -> {
                    //TODO lifer up,claw
                })

                .splineToLinearHeading(new Pose2d(-29, 5, Math.toRadians(315)), Math.toRadians(280))
                .build();

            //Pole to cones
        Trajectory poleCone = mecanumDrive.trajectoryBuilder(startPole.end())
                .addDisplacementMarker(() -> {
                    //TODO lower lifter, release claw
                })
                .splineToSplineHeading(new Pose2d(startPole.end().getX() + 0.0001, startPole.end().getX(), startPole.end().getHeading()), Math.toRadians(170))
                .splineToSplineHeading(new Pose2d(-65, 12, Math.toRadians(180)), Math.toRadians(180))
                .build();

            //cones to pole
        Trajectory conePole = mecanumDrive.trajectoryBuilder(poleCone.end())
                .addTemporalMarker(0, () ->{
                    //TODO set power to zero
                })
                .addDisplacementMarker(() -> {
                    //TODO raise lifter, grab cone
                })
                .splineToSplineHeading(new Pose2d(poleCone.end().getX() + 0.0001, poleCone.end().getX(), poleCone.end().getHeading()), Math.toRadians(170))
                .splineToSplineHeading(new Pose2d(-29, 5, Math.toRadians(315)), Math.toRadians(170))
                .build();

            //pole to parking 1
        Trajectory poleParking1 = mecanumDrive.trajectoryBuilder(conePole.end())
                .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(-36, 24, Math.toRadians(300)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(270)), Math.toRadians(0))
                .build();

            //pole to parking 2

        Trajectory poleParking2 = mecanumDrive.trajectoryBuilder(conePole.end())
                .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(90)), Math.toRadians(90))
                .build();

            //pole to parking 3
        Trajectory poleParking3 = mecanumDrive.trajectoryBuilder(conePole.end())
                .splineToSplineHeading(new Pose2d(conePole.end().getX() + 0.0001, conePole.end().getY(), conePole.end().getHeading()), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(-36, 24, Math.toRadians(270)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-60, 36, Math.toRadians(270)), Math.toRadians(180))
                .build();

            //TODO: add lifer and claw to trajectory code


        waitForStart();
        currentState = State.Start_Pole;
        mecanumDrive.followTrajectoryAsync(startPole);
        while(opModeIsActive()){
            level = roundedAverage(detections);
            telemetry.addData("level", level);
            telemetry.update();
            switch (currentState) {

                case Start_Pole:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.Pole_Cone;
                        mecanumDrive.followTrajectoryAsync(poleCone);
                    }
                    break;

                case Pole_Cone:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.Cone_Pole;
                        mecanumDrive.followTrajectoryAsync(conePole);
                    }
                    break;

                case Cone_Pole:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.Park;
                        if (level == 1) mecanumDrive.followTrajectoryAsync(poleParking1);

                        if (level == 2) mecanumDrive.followTrajectoryAsync(poleParking2);

                        if (level == 3) mecanumDrive.followTrajectoryAsync(poleParking2);
                    }
                    break;

                case Park:
                    if (!mecanumDrive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    break;

                //TODO, add switch case

            }
            mecanumDrive.update();

            /*
            switch(level) {
                case 1:
                    mecanumDrive.driveVelocity(0.6, 0.6, 0.6, 0.6);
                    sleep(800);
                    mecanumDrive.driveVelocity(-0.6, 0.6, 0.6, -0.6);
                    sleep(800);
                    break;
                case 2:
                    mecanumDrive.driveVelocity(0.6, 0.6, 0.6, 0.6);
                    sleep(800);
                    break;
                case 3:
                    mecanumDrive.driveVelocity(0.6, 0.6, 0.6, 0.6);
                    sleep(800);
                    mecanumDrive.driveVelocity(0.6, -0.6, -0.6, 0.6);
                    sleep(750);
                    break;
            }
            if(level != 0){
                break;
            }
            */

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