package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.ActionTaskInterop.actionToTask;
import static org.firstinspires.ftc.teamcode.ActionTaskInterop.taskToAction;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Color;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointLocalizer;

import java.util.ArrayDeque;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(preselectTeleOp = "YugePhart")
public class ColdtonomousBlue extends OpMode {

    private Robot robot;
    private TaskRunner runner;
    private InputManager inputManager;
    private List<LynxModule> hubs;
    private MecanumDrive drive;

    @Override
    public void init() {
        runner = new TaskRunner();
        inputManager = new InputManager();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        if (!Color.getCurrentColor().isPresent()) {
            telemetry.addLine("WARNING: NO COLOR SET! RUN ASetRed OR ASetBlue");
            telemetry.update();
        }

        Pose2d start = new Pose2d(63.7, -15.2, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, start);
        blackboard.put("auto done", false);

        robot = new Robot(hardwareMap, (PinpointLocalizer) drive.localizer, false);
        robot.launcher.setVelCorrectionEnabled(false);

        Supplier<Task> waitRobot = () -> Task.newWithUpdate(() -> robot.notTransitioning());
        Supplier<Task> waitSpunUp =
                () -> Task.newWithUpdate(() -> robot.launcher.isAtTargetRpm()).andThen(new DelayTask(200));
        Supplier<Action> spinUp = () -> taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_END)
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START))
        );
        Supplier<Action> shoot = () -> taskToAction(waitRobot.get().with(waitSpunUp.get())
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START))
                .andThen(new DelayTask(1400))
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_END))
        );
        Action path = drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(taskToAction(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START)))
                .stopAndAdd(shoot.get()) // shoot

                // first set
                .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                .strafeToLinearHeading(new Vector2d(56, -59), Math.toRadians(-77))
                .strafeToLinearHeading(new Vector2d(65, -59), Math.toRadians(-77))
                .strafeToLinearHeading(new Vector2d(65, -60), Math.toRadians(-90))
                .afterTime(0, spinUp.get()) // spin up
                .setReversed(true)
                .splineTo(new Vector2d(57, -20), Math.toRadians(110))
                .stopAndAdd(shoot.get()) // shoot

                // second set
                .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                .setTangent(Math.toRadians(-170))
                .splineToSplineHeading(new Pose2d(49, -24, Math.toRadians(-150)), Math.toRadians(-150))
                .splineTo(new Vector2d(37, -57.5), Math.toRadians(-100))
                .setTangent(Math.toRadians(60))
                .afterTime(0, spinUp.get()) // spin up
                .lineToXLinearHeading(58.5, Math.toRadians(-30))
                .stopAndAdd(shoot.get()) // shoot

                // loose balls
                .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START))) // intake
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(58, -40, Math.toRadians(-90)), Math.toRadians(-90))
                .splineTo(new Vector2d(58, -58), Math.toRadians(-90))
                .endTrajectory()
                .setTangent(Math.toRadians(90))
                .afterTime(0, spinUp.get()) // spin up
                .lineToYLinearHeading(-18, Math.toRadians(-30))
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

                .setTangent(Math.toRadians(-120))
                .lineToX(51)

                .build();
        runner.sendTask(actionToTask(path).andThen(Task.newWithOneshot(() -> {
            blackboard.put("auto done", true);
            blackboard.put("auto pose", drive.localizer.getPose());
        })));
    }

    private final ArrayDeque<Long> lastMs = new ArrayDeque<>();

    private boolean first = true;

    @Override
    public void loop() {
        if (first) {
            first = false;
            runner.sendTask(robot.doInit());
        }

        hubs.forEach(LynxModule::clearBulkCache);

        long time = System.currentTimeMillis();
        lastMs.offerLast(time);
        while (lastMs.size() > 10) {
            lastMs.removeFirst();
        }
        if (lastMs.size() > 1) {
            telemetry.addData("loop ms", (double) (lastMs.getLast() - lastMs.getFirst()) / lastMs.size());
        }
        telemetry.addData("pose", robot.localizer.getPose());

        runner.update();
        robot.update();
    }

}
