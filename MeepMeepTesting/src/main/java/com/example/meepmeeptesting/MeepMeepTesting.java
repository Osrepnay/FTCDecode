package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.function.Supplier;

public class MeepMeepTesting {
    private static Action taskToAction(Object x) {
        return _telemetryPacket -> false;
    }

    private static class Dummy {
        private final Dummy intake = this;

        public Dummy doInit() {
            return this;
        }

        public Dummy deferTransition(Object _x) {
            return this;
        }

        public Dummy with(Object _x) {
            return this;
        }

        public Dummy andThen(Object _x) {
            return this;
        }

        public Dummy setPower(Object _x) {
            return this;
        }
    }

    private static class Task extends Dummy {
        public static Task newWithOneshot(Supplier<Object> _x) {
            return new Task();
        }
    }

    private static class DelayTask extends Dummy {
        public DelayTask(long _x) {}
    }

    private static class Robot {
        public enum Transfer {
            LEFT_BUMPER_START,
            RIGHT_BUMPER_START,
            LEFT_BUMPER_END,
            RIGHT_BUMPER_END
        }
    }

    private class Intake {
        public static final double INTAKE_OFF = 0;
        public static final double INTAKE_ON = 0.8;
        public static final double INTAKE_TRANSFER = 0.3;
        public static final double INTAKE_HOLD = 0.2;
        public static final double INTAKE_BARELYMOVE = 0.1;
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 50, Math.toRadians(180), Math.toRadians(180),
                        5559.450680931914 * (32 * Math.PI / 25.4) / 2000)
                .build();

        Action emptyAction = _telemetryPacket -> false;
        Supplier<Action> shoot = () -> _telemetryPacket -> false;
        Supplier<Action> spinUp = () -> _telemetryPacket -> false;
        Dummy robot = new Dummy();
        Action goontonomous =
                myBot.getDrive().actionBuilder(new Pose2d(-52, 51.5, Math.toRadians(40)))
                        // first set
                        .afterTime(0, taskToAction(robot.doInit()
                                .with(new DelayTask(500)
                                        .andThen(Task.newWithOneshot(() -> robot.intake.setPower(Intake.INTAKE_HOLD)))
                                        .with(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START))
                                )
                        ))
                        .waitSeconds(0.6)
                        .setTangent(Math.toRadians(-50))
                        .lineToX(-30)
                        .stopAndAdd(shoot.get())

                        // pickup, score second set
                        .afterTime(0.0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                        .splineTo(new Vector2d(-13, 48), Math.toRadians(90))
                        .splineTo(new Vector2d(-13, 51), Math.toRadians(90))
                        .setTangent(0)
                        .splineTo(new Vector2d(-4, 59), Math.toRadians(90))
                        .waitSeconds(0.6)
                        .afterTime(0, spinUp.get())
                        .strafeToLinearHeading(new Vector2d(-16, 23), Math.toRadians(30))
                        .stopAndAdd(shoot.get())

                        // third set
                        .afterTime(0.3, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                        .splineTo(new Vector2d(10, 40), Math.toRadians(73))
                        .splineTo(new Vector2d(13, 56), Math.toRadians(78))
                        .afterTime(0.3, spinUp.get())
                        .strafeToLinearHeading(new Vector2d(-16, 23), Math.toRadians(30))
                        .stopAndAdd(shoot.get())

                        // fourth/final set
                        .afterTime(0.6, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                        .splineTo(new Vector2d(32, 40), Math.toRadians(64))
                        .splineTo(new Vector2d(35, 56), Math.toRadians(70))
                        .afterTime(0.6, spinUp.get())
                        .setReversed(true)
                        .splineTo(new Vector2d(-16, 23), Math.toRadians(180))
                        .stopAndAdd(shoot.get())

                        .setTangent(Math.toRadians(45))
                        .lineToX(-8)

                        .build();

        Action special =
                myBot.getDrive()
                        .actionBuilder(new Pose2d(63.7, 15.2, Math.toRadians(-90)))
                        .stopAndAdd(taskToAction(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START)))
                        .stopAndAdd(shoot.get()) // shoot

                        // first set
                        .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                        .strafeToLinearHeading(new Vector2d(56, 59), Math.toRadians(77))
                        .strafeToLinearHeading(new Vector2d(65, 59), Math.toRadians(77))
                        .strafeToLinearHeading(new Vector2d(65, 60), Math.toRadians(90))
                        .afterTime(0, spinUp.get()) // spin up
                        .setReversed(true)
                        .splineTo(new Vector2d(57, 20), Math.toRadians(-110))
                        .stopAndAdd(shoot.get()) // shoot

                        // second set
                        .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                        .setTangent(Math.toRadians(170))
                        .splineToSplineHeading(new Pose2d(49, 24, Math.toRadians(150)), Math.toRadians(150))
                        .splineTo(new Vector2d(37, 57.5), Math.toRadians(100))
                        .setTangent(Math.toRadians(-60))
                        .afterTime(0, spinUp.get()) // spin up
                        .lineToXLinearHeading(58.5, Math.toRadians(30))
                        .stopAndAdd(shoot.get()) // shoot

                        // loose balls
                        .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(58, 40, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(58, 58), Math.toRadians(90))
                        .endTrajectory()
                        .setTangent(Math.toRadians(-90))
                        .afterTime(0, spinUp.get()) // spin up
                        .lineToYLinearHeading(18, Math.toRadians(30))
                        .stopAndAdd(shoot.get()) // shoot

                        /*
                        .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(56, 40, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(56, 58), Math.toRadians(90))
                        .endTrajectory()
                        .afterTime(0, spinUp.get()) // spin up
                        .setTangent(Math.toRadians(-90))
                        .lineToYLinearHeading(18, Math.toRadians(30))
                        .stopAndAdd(shoot.get()) // shoot
                         */

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