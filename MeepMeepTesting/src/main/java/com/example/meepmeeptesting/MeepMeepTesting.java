package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
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
                .setConstraints(70, 50, Math.toRadians(180), Math.toRadians(180),
                        5559.450680931914 * (32 * Math.PI / 25.4) / 2000)
                .build();

        Action emptyAction = _telemetryPacket -> false;
        Action goontonomous =
                myBot.getDrive().actionBuilder(new Pose2d(-52, 51.5, Math.toRadians(40)))
                        // first set
                        .afterTime(0, emptyAction)
                        .waitSeconds(0.6)
                        .setTangent(Math.toRadians(-50))
                        .lineToX(-30)
                        .stopAndAdd(emptyAction)

                        // pickup, score second set
                        .afterTime(0.3, emptyAction)
                        .splineTo(new Vector2d(-13, 48), Math.toRadians(90))
                        .splineTo(new Vector2d(-13, 53), Math.toRadians(90))
                        .setTangent(0)
                        .splineTo(new Vector2d(-4, 59), Math.toRadians(90))
                        .waitSeconds(0.6)
                        .afterTime(0, emptyAction)
                        .strafeToLinearHeading(new Vector2d(-16, 23), Math.toRadians(30))
                        .stopAndAdd(emptyAction)

                        // third set
                        .afterTime(0.8, emptyAction)
                        .splineTo(new Vector2d(8, 40), Math.toRadians(65))
                        .splineTo(new Vector2d(11, 48), Math.toRadians(70))
                        .afterTime(0.3, emptyAction)
                        .strafeToLinearHeading(new Vector2d(-16, 23), Math.toRadians(30))
                        .stopAndAdd(emptyAction)

                        // fourth/final set
                        .afterTime(1.3, emptyAction)
                        .splineTo(new Vector2d(32, 40), Math.toRadians(60))
                        .splineTo(new Vector2d(35, 48), Math.toRadians(75))
                        .afterTime(0.6, emptyAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(-16, 23), Math.toRadians(180))
                        .stopAndAdd(emptyAction)

                        .setTangent(Math.toRadians(45))
                        .lineToX(-13)

                        .build();

        Action special =
                myBot.getDrive()
                        .actionBuilder(new Pose2d(63.7, 15.2, Math.toRadians(-90)))
                        .stopAndAdd(emptyAction) // shoot

                        // first set
                        .afterTime(0, emptyAction) // intake
                        .strafeToLinearHeading(new Vector2d(50, 60), Math.toRadians(75))
                        .strafeToLinearHeading(new Vector2d(63, 62), Math.toRadians(90))
                        .afterTime(0, emptyAction) // spin up
                        .setReversed(true)
                        .splineTo(new Vector2d(57, 20), Math.toRadians(-110))
                        .stopAndAdd(emptyAction) // shoot

                        // second set
                        .afterTime(0, emptyAction) // intake
                        .setTangent(Math.toRadians(170))
                        .splineToSplineHeading(new Pose2d(48, 25, Math.toRadians(140)), Math.toRadians(140))
                        .splineTo(new Vector2d(35, 55), Math.toRadians(100))
                        .setTangent(Math.toRadians(-60))
                        .afterTime(0, emptyAction) // spin up
                        .lineToXLinearHeading(57, Math.toRadians(30))
                        .stopAndAdd(emptyAction) // shoot

                        // loose balls
                        .afterTime(0, emptyAction) // intake
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(54, 40, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(54, 59), Math.toRadians(90))
                        .endTrajectory()
                        // .setTangent(Math.toRadians(-65))
                        .afterTime(0, emptyAction) // spin up
                        .setTangent(Math.toRadians(-94))
                        .lineToYLinearHeading(18, Math.toRadians(30))
                        .stopAndAdd(emptyAction) // shoot

                        .setTangent(Math.toRadians(120))
                        .lineToX(51)

                        .build();

        myBot.runAction(special);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}