package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Trajectories {
    private SampleMecanumDrive drive;
    public static Pose2d startPose;
    public Trajectory startPole;
    public Trajectory poleCone;
    public Trajectory conePole;
    public enum Start {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public Trajectories(SampleMecanumDrive drive, Start startpos) {
        this.drive = drive;

        switch (startpos) {
            case RED_LEFT:
                startPose = new Pose2d(-36, -65, Math.toRadians(0));

                startPole = drive.trajectoryBuilder(startPose, Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-36, -17), Math.toRadians(85))
                        .splineToSplineHeading(
                                new Pose2d(-28.5, -4.5, Math.toRadians(45)), Math.toRadians(40),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                poleCone = drive.trajectoryBuilder(startPole.end(), Math.toRadians(250))
                        .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(180))
                        .splineToConstantHeading(
                                new Vector2d(-65, -12), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                conePole = drive.trajectoryBuilder(poleCone.end(), 0)
                        .splineToConstantHeading(new Vector2d(-55, -12), 0)
                        .splineToConstantHeading(new Vector2d(-45, -12), 0)
                        .splineToSplineHeading(new Pose2d(-28.5, -4.5, Math.toRadians(45)), Math.toRadians(50))
                        .build();
                break;
        }
    }
}
