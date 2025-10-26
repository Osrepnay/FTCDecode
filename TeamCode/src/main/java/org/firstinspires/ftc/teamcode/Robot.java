package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.ArrayDeque;
import java.util.Optional;

public class Robot {
    public enum State {
        IDLE(false),
        INTAKING(false),
        PRIMED(true),
        SHOOTING(true);

        public final boolean spunUp;

        State(boolean spunUp) {
            this.spunUp = spunUp;
        }
    }

    public enum Transfer {
        LEFT_BUMPER_START,
        RIGHT_BUMPER_START,
        RIGHT_BUMPER_END
    }

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Latch latch;
    public final Launcher launcher;
    public final Camera camera;

    private State state = State.IDLE;
    private final ArrayDeque<State> futureStates = new ArrayDeque<>();
    private boolean forceUnlock = false;

    public Robot(HardwareMap hardwareMap) {
        // ??
        VoltageSensor voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        launcher = new Launcher(hardwareMap, voltageSensor);
        camera = new Camera(hardwareMap);
    }

    public void performTransition(TaskRunner runner, Transfer transfer) {
        Task task = Task.empty();
        State targetState = null;
        // state the task hypothetically would be at
        State futureState = futureStates.isEmpty() ? state : futureStates.getLast();
        switch (futureState) {
            case IDLE:
                switch (transfer) {
                    case LEFT_BUMPER_START:
                        targetState = State.INTAKING;
                        task = latch.close()
                                .andThen(Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_ON)));
                        break;
                    case RIGHT_BUMPER_START:
                        targetState = State.PRIMED;
                        // TODO lock
                        task = Task.newWithOneshot(() -> launcher.setTargetRpm(Launcher.LAUNCH_RPM));
                        break;
                }
                break;
            case INTAKING:
                if (transfer == Transfer.RIGHT_BUMPER_START) {
                    targetState = State.IDLE;
                    task = Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_OFF))
                            .andThen(new DelayTask(300))
                            .andThen(latch.open());
                }
                break;
            case PRIMED:
                switch (transfer) {
                    case LEFT_BUMPER_START:
                        targetState = State.IDLE;
                        task = Task.newWithOneshot(launcher::killPower);
                        break;
                    case RIGHT_BUMPER_START:
                        targetState = State.SHOOTING;
                        task = launcher.waitForSpinUp()
                                .andThen(Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_TRANSFER)));
                        break;
                }
                break;
            case SHOOTING:
                if (transfer == Transfer.RIGHT_BUMPER_END) {
                    targetState = State.IDLE;
                    task = Task.newWithOneshot(() -> {
                        launcher.killPower();
                        intake.setPower(Intake.INTAKE_OFF);
                    });
                }
        }
        if (targetState != null) {
            futureStates.offerLast(targetState);
            State finalTargetState = targetState;
            runner.sendTask(task
                    .andThen(Task.newWithOneshot(() -> {
                        state = finalTargetState;
                        futureStates.removeFirst();
                    }))
                    .withResources(this)
            );
        }
    }

    public void update() {
        launcher.update();
        camera.update();

        if (notTransitioning() && !forceUnlock && state.spunUp) {
            Optional<Double> camHeading = camera.getBearing();
            if (camHeading.isPresent()) {
                drivetrain.lockHeading(camHeading.get());
            } else {
                drivetrain.unlockHeading();
            }
        } else {
            drivetrain.unlockHeading();
        }

        if (notTransitioning() && state.spunUp) {
            camera.getRange().ifPresent(launcher::setTargetRpmByDistance);
        }
    }

    public State getState() {
        return state;
    }

    public boolean notTransitioning() {
        return futureStates.isEmpty();
    }

    public void toggleUnlockOverride() {
        forceUnlock = !forceUnlock;
    }
}
