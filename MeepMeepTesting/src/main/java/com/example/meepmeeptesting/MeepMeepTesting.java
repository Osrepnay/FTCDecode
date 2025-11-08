package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7561.184699738115 * 95.0 / 48427)
                .build();

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(new Pose2d(-51, 51, Math.toRadians(135)))
                        // first set
                        .lineToX(-23)
                        .endTrajectory()

                        // pickup, score second set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-13, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-13, 44))
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .endTrajectory()

                        // third set
                        .setTangent(Math.toRadians(-30))
                        .splineToSplineHeading(new Pose2d(4, 17, Math.toRadians(30)), Math.toRadians(30))
                        .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(12, 44))
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .endTrajectory()

                        // fourth/final set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(12, 17, Math.toRadians(10)), Math.toRadians(8))
                        .splineToSplineHeading(new Pose2d(34.5, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(34.5, 44))
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .endTrajectory()

                        .setTangent(Math.toRadians(45))
                        .lineToX(-13)
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}