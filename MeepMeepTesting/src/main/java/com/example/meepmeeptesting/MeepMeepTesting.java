package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    enum State {
        Start_Pole,   // Start -> first pole (raise arm)
        Pole_Cone,   // first pole -> cones (lower arm)
        Cone_Pole,         // cones -> pole (raise arm)
        Pole_Parking1,   // pole -> parking 1
        Pole_Parking2,         // pole -> parking 2
        Pole_Parking3,         // pole -> parking 3
        Drop,           //Drop Cone
        Grab,           //Grab Cone
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;
    public static void main(String[] args) {












        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(283.4867829353258), Math.toRadians(273.362373818), 12)
                .followTrajectorySequence(drive ->
/*
                         drive.trajectorySequenceBuilder(new Pose2d(-36, 65, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-29, 5, Math.toRadians(315)), Math.toRadians(280))
                                .build()



 */

                        /*

                        drive.trajectorySequenceBuilder(new Pose2d(-29, 5, Math.toRadians(315)))
                                .splineToSplineHeading(new Pose2d(-29.0001, 5.001, Math.toRadians(315)), Math.toRadians(170))
                                .splineToSplineHeading(new Pose2d(-65, 12, Math.toRadians(180)), Math.toRadians(180))
                                .build()


                         */

                        /*   Pole -> parking 3

                         */
                        /*
                          drive.trajectorySequenceBuilder(new Pose2d(-29, 5, Math.toRadians(315)))
                                  .splineToSplineHeading(new Pose2d(-29.0001, 5.0001, Math.toRadians(315)), Math.toRadians(100))
                                  .splineToSplineHeading(new Pose2d(-36, 24, Math.toRadians(270)), Math.toRadians(90))
                                  .splineToSplineHeading(new Pose2d(-60, 36, Math.toRadians(270)), Math.toRadians(180))
                                  .build()


                         */

                        /* Pole -> parking 2


                                drive.trajectorySequenceBuilder(new Pose2d(-29, 5, Math.toRadians(315)))
                                        .splineToSplineHeading(new Pose2d(-29.0001, 5.0001, Math.toRadians(315)), Math.toRadians(100))
                                        .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(90)), Math.toRadians(90))
                                        .build()

                         */

                        // Pole -> parking 3
                                drive.trajectorySequenceBuilder(new Pose2d(-29, 5, Math.toRadians(315)))
                                        .splineToSplineHeading(new Pose2d(-29.0001, 5.0001, Math.toRadians(315)), Math.toRadians(100))
                                        .splineToSplineHeading(new Pose2d(-36, 24, Math.toRadians(300)), Math.toRadians(90))
                                        .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(270)), Math.toRadians(0))
                                        .build()






                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}