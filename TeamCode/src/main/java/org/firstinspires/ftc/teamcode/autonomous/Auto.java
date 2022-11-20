package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Auto extends LinearOpMode{
    OpenCvWebcam webcam;
    private ColorDetector detector;
    private int level;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightRear"));

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
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for Start");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            level = detector.getLevel();
            telemetry.addData("level", level);
            telemetry.addData("values", detector.getValues());
            telemetry.update();

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
                    sleep(700);
                    mecanumDrive.driveVelocity(0.6, -0.6, -0.6, 0.6);
                    sleep(800);
                    break;
            }
            if(level != 0){
                break;
            }
        }
    }
}