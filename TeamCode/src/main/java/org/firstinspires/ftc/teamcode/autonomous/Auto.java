package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Claw;
import org.firstinspires.ftc.teamcode.Robot.Lifter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Auto extends LinearOpMode {
    private enum State {
        Start_Pole,
        Pole_Cone,
        Cone_Pole,
        Lift,
        Park,
        IDLE
    }
    private OpenCvWebcam webcam;
    private ColorDetector detector;
    private int[] detections = new int[100];
    private int level;
    private Trajectories trajectories;

    private State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        Lifter lifter = new Lifter(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

        trajectories = new Trajectories(drive, Trajectories.Start.RED_LEFT); //make 4 different opmodes for each start position
        currentState = State.Start_Pole;
        claw.close();
        drive.followTrajectoryAsync(trajectories.startPole);
        lifter.setTargetPosition(3);
    }

    private int roundedAverage(int[] arr) {
        double sum = 0;
        for(int i = 0; i < arr.length; i++){
            sum+=arr[i];
        }
        return (int)(Math.round(sum/arr.length));
    }
}