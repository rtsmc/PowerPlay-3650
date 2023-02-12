package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Park2 extends LinearOpMode {
    OpenCvWebcam webcam;
    private ColorDetector detector;
    int[] detections = new int[100];
    private int level;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(0));
        //opencv
        detector = new ColorDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.addLine("Waiting for Start");
        telemetry.update();
        int i = 0;
        while (!isStarted()) {
            if (i == 99) {
                i = 0;
            }
            detections[i] = detector.getLevel();
            i++;
            telemetry.addData("average of detections", roundedAverage(detections));
            telemetry.update();
        }
    }

    private int roundedAverage(int[] arr) {
        double sum = 0;
        for (int j : arr) {
            sum += j;
        }
        return (int)(Math.round(sum/arr.length));
    }
}
