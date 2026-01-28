package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;
import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointLocalizer;

import java.util.ArrayDeque;
import java.util.Optional;

@Config
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
        LEFT_BUMPER_END,
        RIGHT_BUMPER_END
    }

    public long LL_INTERVAL_MS = 100;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Latch latch;
    public final Launcher launcher;
    public final Localizer localizer;
    public final Limelight3A limelight;

    private State state = State.IDLE;
    private final ArrayDeque<State> futureStates = new ArrayDeque<>();
    private boolean forceUnlock = false;
    // TODO this only does things on rpm control for now
    // rememer to integrate when fix the drivetrain update weirdness
    private boolean stopAutoRpm = false;
    private boolean slowShoot = false;
    private double goalDist = -1;
    private boolean updateLocalizer = true;

    public Robot(HardwareMap hardwareMap) {
        // pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 235.17, -1619.83, AngleUnit.DEGREES, 90));
        this(hardwareMap, new Pose2d(
                .375 + 181.525 / 25.4,
                .375 + 184 / 25.4 * Color.currentAsMult(),
                Math.toRadians(90 * Color.currentAsMult())
        ));
    }

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        this(hardwareMap, new PinpointLocalizer(
                hardwareMap,
                MecanumDrive.PARAMS.inPerTick,
                startPose
        ), true);
    }

    public Robot(HardwareMap hardwareMap, Localizer existingLocalizer, boolean updateLocalizer) {
        this.updateLocalizer = updateLocalizer;
        localizer = existingLocalizer;
        VoltageSensor voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        drivetrain = new Drivetrain(hardwareMap, voltageSensor);
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        launcher = new Launcher(hardwareMap, voltageSensor);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public Task doInit() {
        return latch.doClose()
                .with(launcher.doInit())
                .with(Task.newWithOneshot(limelight::start));
    }

    // TODO this whole thing is kinda a mess
    // i think best thing right now might be a transfer queue? and everything is deferred
    // problem with that is that it's a lot more annoying to look into future - s.spunUp
    public Task deferTransition(Transfer transfer) {
        return Task.defer(() -> transition(transfer).orElse(Task.empty()))
                .withResources(this);
    }

    public void performTransition(TaskRunner runner, Transfer transfer) {
        runner.sendTask(transition(transfer).orElse(Task.empty()));
    }

    // TODO:
    // no transition queue. transition method sets task immediately upon task run. auto-style transition chaining
    // handled some other way?
    public Optional<Task> transition(Transfer transfer) {
        Task task = Task.empty();
        State targetState = null;
        // state the task hypothetically would be at
        State futureState = futureStates.isEmpty() ? state : futureStates.getLast();
        switch (futureState) {
            case IDLE:
                switch (transfer) {
                    case LEFT_BUMPER_START:
                        targetState = State.INTAKING;
                        task = Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_ON));
                        break;
                    case RIGHT_BUMPER_START:
                        targetState = State.PRIMED;
                        task = Task.newWithOneshot(() -> launcher.setTargetRpm(launcher.fallbackRpm));
                        break;
                }
                break;
            case INTAKING:
                if (transfer == Transfer.LEFT_BUMPER_END) {
                    targetState = State.IDLE;
                    task = Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_HOLD));
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
                                .andThen(Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_OFF)))
                                .andThen(latch.doOpen())
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
                    }).with(latch.doClose());
                }
                break;
        }
        if (targetState != null) {
            State finalTargetState = targetState;
            return Optional.of(
                    Task.newWithOneshot(() -> futureStates.offerLast(finalTargetState))
                            .andThen(task)
                            .andThen(Task.newWithOneshot(() -> {
                                state = finalTargetState;
                                futureStates.removeFirst();
                            }))
                            .withResources(this)
            );
        } else {
            return Optional.empty();
        }
    }

    private double moveCloser(double start, double end, double proportion) {
        return start + (end - start) * proportion;
    }

    private long lastLLUpdate = 0;

    public void update() {
        long time = System.currentTimeMillis();

        launcher.update();
        if (updateLocalizer) {
            localizer.update();
        }
        double heading = localizer.getPose().heading.toDouble();
        drivetrain.update(heading - Math.PI / 2 * Color.currentAsMult());

        double xMm = localizer.getPose().position.x * 25.4;
        double yMm = localizer.getPose().position.y * 25.4;

        /*
        if (Math.sqrt(Math.pow(pinpoint.getVelX(DistanceUnit.MM), 2) + Math.pow(pinpoint.getVelY(DistanceUnit.MM), 2)
        ) < 40
                && pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) < 2) {
            // limelight.updateRobotOrientation(Math.toDegrees(heading) - 90);
            LLResult result = limelight.getLatestResult();
            if (time - lastLLUpdate > LL_INTERVAL_MS && result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose.getPosition().x != 0 || botpose.getPosition().y != 0) {
                    System.out.println(botpose);
                    Pose2D newPos = new Pose2D(
                            DistanceUnit.MM,
                            moveCloser(xMm, botpose.getPosition().x * 1000, 0.3),
                            moveCloser(yMm, botpose.getPosition().y * 1000, 0.3),
                            AngleUnit.RADIANS,
                            heading
                    );
                    // pinpoint.setPosition(newPos);
                }
            }
        }
         */

        Vector2d launcherTargetMm = new Vector2d(
                -(70.75 - 2.5) * 25.4,
                (70.75 - 2.5 - 6) * 25.4 * Color.currentAsMult()
        );

        double lockRad = Math.atan2(
                launcherTargetMm.y - yMm,
                launcherTargetMm.x - xMm
        );
        launcher.setTurretRadians(lockRad - (heading + Math.PI));

        goalDist = Math.sqrt(Math.pow(launcherTargetMm.y - yMm, 2) + Math.pow(launcherTargetMm.x - xMm, 2));
        if (!stopAutoRpm && state.spunUp && getNextState().map(s -> s.spunUp).orElse(true)) {
            launcher.setByDistance(goalDist);
        }
    }

    public void resetPos() {
        if (Color.getCurrentColor().orElse(Color.RED) == Color.RED) {
            // localizer.setPose(new Pose2d(new Vector2d(63.2, 61.1), localizer.getPose().heading));
            localizer.setPose(new Pose2d(new Vector2d(63.2, -61.1), Math.PI / 2));
        } else {
            localizer.setPose(new Pose2d(new Vector2d(63.2, 61.1), -Math.PI / 2));
        }
    }

    public State getState() {
        return state;
    }

    public Optional<State> getNextState() {
        return Optional.ofNullable(futureStates.peekFirst());
    }

    public boolean notTransitioning() {
        return futureStates.isEmpty();
    }

    public void toggleUnlockOverride() {
        forceUnlock = !forceUnlock;
    }

    public void disableAutoRpm() {
        stopAutoRpm = true;
    }

    public void enableAutoRpm() {
        stopAutoRpm = false;
    }

    public boolean isAutoRpmStopped() {
        return stopAutoRpm;
    }

    public void setSlowShoot(boolean slowShoot) {
        this.slowShoot = slowShoot;
    }

    public boolean isSlowShoot() {
        return slowShoot;
    }

    public double getGoalDist() {
        return goalDist;
    }
}
