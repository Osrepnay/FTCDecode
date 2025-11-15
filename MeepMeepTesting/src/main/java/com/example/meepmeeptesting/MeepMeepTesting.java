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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7561.184699738115 * 95.0 / 48427)
                .build();

        Action emptyAction = _telemetryPacket -> false;
        Action goontonomous =
                myBot.getDrive()
                        .actionBuilder(new Pose2d(-51, 51, Math.toRadians(135)))
                        // first set
                        .afterTime(0, emptyAction)
                        .waitSeconds(0.5)
                        .lineToX(-23)
                        .stopAndAdd(emptyAction)

                        // pickup, score second set
                        .afterTime(0.4, emptyAction)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-13, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-13, 48))
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-1, 56), Math.toRadians(90))
                        .afterTime(1, emptyAction)
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .stopAndAdd(emptyAction)

                        // third set
                        .afterTime(1.5, emptyAction)
                        .setTangent(Math.toRadians(-30))
                        .splineToSplineHeading(new Pose2d(4, 17, Math.toRadians(30)), Math.toRadians(30))
                        .splineToSplineHeading(new Pose2d(11, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(11, 48))
                        .afterTime(0, emptyAction)
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .stopAndAdd(emptyAction)

                        // fourth/final set
                        .afterTime(1.6, emptyAction)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(12, 17, Math.toRadians(10)), Math.toRadians(8))
                        .splineToSplineHeading(new Pose2d(34.5, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(34.5, 48))
                        .afterTime(0, emptyAction)
                        .strafeToLinearHeading(new Vector2d(-23, 23), Math.toRadians(135))
                        .stopAndAdd(emptyAction)

                        .setTangent(Math.toRadians(45))
                        .lineToX(-13)
                        .build();

        final double shootAngle = Math.toRadians(161);
        Vector2d shootLoc = new Vector2d(55, 21);
        Action special =
                myBot.getDrive()
                        .actionBuilder(new Pose2d(60, 19, Math.toRadians(180)))
                        // first set
                        .afterTime(0, emptyAction)
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(shootLoc, shootAngle)
                        .stopAndAdd(emptyAction)

                        // pickup, score second set
                        .afterTime(0.2, emptyAction)
                        .splineToSplineHeading(new Pose2d(34, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(34, 56))
                        .waitSeconds(5)
                        .afterTime(0, emptyAction)
                        .setReversed(true)
                        .splineTo(shootLoc, shootAngle - Math.PI)
                        .stopAndAdd(emptyAction)

                        // third set
                        .afterTime(1.2, emptyAction)
                        .splineToSplineHeading(new Pose2d(10, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(10, 56))
                        .afterTime(0, emptyAction)
                        .setReversed(true)
                        .splineTo(shootLoc, shootAngle - Math.PI)
                        .stopAndAdd(emptyAction)

                        .lineToX(40)
                        .build();

        myBot.runAction(goontonomous);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}