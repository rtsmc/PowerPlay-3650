package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class Left extends LinearOpMode {
    enum State {
        Start_Pole,
        Pole_Cone,
        Cone_Pole,
        Grab,
        Park,
        IDLE
    }
    private OpenCvWebcam webcam;
    private final int[] detections = new int[100];
    private int level;
    @Override
    public void runOpMode() throws InterruptedException {
        State currentState;
        int cycle = 0;
        Lifter lifter = new Lifter(hardwareMap);
        CRServo claw = hardwareMap.get(CRServo.class, "clawServo");
        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(0));
        ColorDetector detector = new ColorDetector();

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        Trajectory startPole = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 0), Math.toRadians(85))
                .splineToConstantHeading(
                        new Vector2d(-30.5, -1), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory poleCone = drive.trajectoryBuilder(startPole.end(), Math.toRadians(250))
                .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(-65, -12), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory conePole = drive.trajectoryBuilder(poleCone.end(), 0)
                .splineToSplineHeading(new Pose2d(-35, 0, Math.toRadians(0)), 90)
                .splineToConstantHeading(new Vector2d(-31, -1), 90)
                .build();

        Trajectory poleParking1 = drive.trajectoryBuilder(conePole.end(), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 0), Math.toRadians(270))
                .splineTo(new Vector2d(-60, -12), Math.toRadians(170))
                .build();

        Trajectory poleParking2 = drive.trajectoryBuilder(conePole.end(), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 0), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-35, -15), Math.toRadians(270))
                .build();

        Trajectory poleParking3 = drive.trajectoryBuilder(conePole.end(), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-35, 0), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-35, -12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
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
                        currentState = State.Cone_Pole;
//                        claw.setPower(-0.3);
//                        sleep(900);
//                        claw.setPower(0);
//                        lifter.setTargetPosition(4);
//                        poleCone = drive.trajectoryBuilder(conePole.end(), Math.toRadians(250))
//                                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(180))
//                                .splineToConstantHeading(
//                                        new Vector2d(-65, -12), Math.toRadians(180),
//                                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                                )
//                                .build();
                    }
                    break;
                case Pole_Cone:
                    if(!drive.isBusy()){
                        claw.setPower(0.3);
                        sleep(800);
                        lifter.setTargetPosition(3);
                        currentState = State.Grab;
                    }
                    break;
                case Grab:
                    claw.setPower(0.3);
                    if(lifter.getPositions()[0] > 3000){
                        currentState = State.Cone_Pole;
                        drive.followTrajectoryAsync(conePole);
                    }
                    break;
                case Cone_Pole:
                    if(!drive.isBusy()){
                        claw.setPower(-0.3);
                        sleep(900);
                        claw.setPower(0);
                        cycle++;
                        if(cycle == 1){
                            lifter.setTargetPosition(0);
                            currentState = State.Park;
                            if (level == 1) drive.followTrajectoryAsync(poleParking1);

                            if (level == 2) drive.followTrajectoryAsync(poleParking2);

                            if (level == 3) drive.followTrajectoryAsync(poleParking3);
                        } else {
                            currentState = State.Pole_Cone;
                            lifter.setTargetPosition(4+cycle);
                            drive.followTrajectoryAsync(poleCone);
                        }
                    }
                    break;
                case Park:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
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