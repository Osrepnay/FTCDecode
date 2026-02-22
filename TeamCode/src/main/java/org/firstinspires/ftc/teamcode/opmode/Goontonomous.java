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
import org.firstinspires.ftc.teamcode.Intake;
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
public class Goontonomous extends OpMode {

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

        Pose2d start = new Pose2d(-52, 51.5, Math.toRadians(40));
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
        telemetry.addData("target rpm", robot.launcher.getTargetRpm());

        runner.update();
        robot.update();
    }

}
