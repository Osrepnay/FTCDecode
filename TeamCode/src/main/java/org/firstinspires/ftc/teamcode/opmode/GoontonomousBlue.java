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

import java.util.ArrayDeque;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(preselectTeleOp = "YugePhart")
public class GoontonomousBlue extends OpMode {

    private Robot robot;
    private TaskRunner runner;
    private InputManager inputManager;
    private List<LynxModule> hubs;
    private MecanumDrive drive;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.disableCamera();
        robot.launcher.fallbackRpm = 855;
        runner = new TaskRunner();
        inputManager = new InputManager();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        if (!Color.getCurrentColor().isPresent()) {
            telemetry.addLine("WARNING: NO COLOR SET! RUN ASetRed OR ASetBlue");
            telemetry.update();
        }

        Pose2d start = new Pose2d(-51, -51, Math.toRadians(-135));
        drive = new MecanumDrive(hardwareMap, start);

        Supplier<Task> waitRobot = () -> Task.newWithUpdate(() -> robot.notTransitioning());
        Supplier<Task> waitSpunUp =
                () -> Task.newWithUpdate(() -> robot.launcher.isAtTargetRpm()).andThen(new DelayTask(1300));
        Supplier<Action> spinUp = () -> taskToAction(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START)
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START))
        );
        Supplier<Action> shoot = () -> taskToAction(waitRobot.get()
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START).with(waitSpunUp.get()))
                .andThen(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_END))
        );
        Action path = drive.actionBuilder(drive.localizer.getPose())
                // first set
                .afterTime(0, taskToAction(robot.deferTransition(Robot.Transfer.RIGHT_BUMPER_START)))
                .waitSeconds(0.5)
                .lineToX(-23)
                .stopAndAdd(shoot.get())

                // pickup, score second set
                .afterTime(0.4, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-13, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-13, -48))
                .afterTime(0, spinUp.get())
                .strafeToLinearHeading(new Vector2d(-23, -23), Math.toRadians(-135))
                .stopAndAdd(shoot.get())

                // third set
                .afterTime(1.5, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(4, -17, Math.toRadians(-30)), Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(11, -48))
                .afterTime(0, spinUp.get())
                .strafeToLinearHeading(new Vector2d(-23, -23), Math.toRadians(-135))
                .stopAndAdd(shoot.get())

                // fourth/final set
                .afterTime(1.6, taskToAction(robot.deferTransition(Robot.Transfer.LEFT_BUMPER_START)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, -17, Math.toRadians(-10)), Math.toRadians(-8))
                .splineToSplineHeading(new Pose2d(34.5, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(34.5, -48))
                .afterTime(0, spinUp.get())
                .strafeToLinearHeading(new Vector2d(-23, -23), Math.toRadians(-135))
                .stopAndAdd(shoot.get())

                .setTangent(Math.toRadians(-45))
                .lineToX(-13)
                .build();
        runner.sendTask(actionToTask(path));
    }

    private final ArrayDeque<Long> lastMs = new ArrayDeque<>();

    private boolean first = true;

    @Override
    public void loop() {
        if (first) {
            first = false;
            runner.sendTask(robot.init());
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

        runner.update();
        robot.update();
    }

}
