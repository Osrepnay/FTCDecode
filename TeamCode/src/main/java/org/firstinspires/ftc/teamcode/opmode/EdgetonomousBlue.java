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
import org.firstinspires.ftc.teamcode.RobotOld;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayDeque;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(preselectTeleOp = "YugePhart")
public class EdgetonomousBlue extends OpMode {

    private RobotOld robot;
    private TaskRunner runner;
    private InputManager inputManager;
    private List<LynxModule> hubs;
    private MecanumDrive drive;

    @Override
    public void init() {
        robot = new RobotOld(hardwareMap);
        robot.disableCamera();
        robot.launcher.fallbackRpm = 1040;
        runner = new TaskRunner();
        inputManager = new InputManager();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        if (!Color.getCurrentColor().isPresent()) {
            telemetry.addLine("WARNING: NO COLOR SET! RUN ASetRed OR ASetBlue");
            telemetry.update();
        }

        Pose2d start = new Pose2d(60, -19, Math.toRadians(-180));
        drive = new MecanumDrive(hardwareMap, start);

        Supplier<Task> waitRobot = () -> Task.newWithUpdate(() -> robot.notTransitioning());
        Supplier<Task> waitSpunUp =
                () -> Task.newWithUpdate(() -> robot.launcher.isAtTargetRpm()).andThen(new DelayTask(1600));
        Supplier<Action> spinUp = () -> taskToAction(robot.deferTransition(RobotOld.Transfer.RIGHT_BUMPER_START)
                .andThen(robot.deferTransition(RobotOld.Transfer.RIGHT_BUMPER_START))
        );
        Supplier<Action> shoot = () -> taskToAction(waitRobot.get()
                .andThen(robot.deferTransition(RobotOld.Transfer.RIGHT_BUMPER_START).with(waitSpunUp.get()))
                .andThen(robot.deferTransition(RobotOld.Transfer.RIGHT_BUMPER_END))
        );

        final double shootAngle = Math.toRadians(-161);
        Vector2d shootLoc = new Vector2d(55, -21);
        Action path = drive.actionBuilder(drive.localizer.getPose())
                // first set
                .afterTime(0, taskToAction(robot.deferTransition(RobotOld.Transfer.RIGHT_BUMPER_START)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(shootLoc, shootAngle)
                .stopAndAdd(shoot.get())

                // pickup, score second set
                .afterTime(0.2, taskToAction(robot.deferTransition(RobotOld.Transfer.LEFT_BUMPER_START)))
                .splineToSplineHeading(new Pose2d(34, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(34, -56))
                .waitSeconds(5)
                .afterTime(0, spinUp.get())
                .setReversed(true)
                .splineTo(shootLoc, shootAngle + Math.PI)
                .stopAndAdd(shoot.get())

                // third set
                .afterTime(1.2, taskToAction(robot.deferTransition(RobotOld.Transfer.LEFT_BUMPER_START)))
                .splineToSplineHeading(new Pose2d(10, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(10, -56))
                .afterTime(0, spinUp.get())
                .setReversed(true)
                .splineTo(shootLoc, shootAngle + Math.PI)
                .stopAndAdd(shoot.get())

                .lineToX(40)
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
