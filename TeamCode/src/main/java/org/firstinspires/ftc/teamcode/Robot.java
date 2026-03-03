package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointLocalizer;

import java.util.ArrayDeque;
import java.util.Optional;

@Config
public class Robot {
    public enum State {
        IDLE(false, false, false),
        INTAKING(false, false, false),
        PRIMED(true, false, false),
        SHOOTING(true, false, true);

        public final boolean spunUp;
        public final boolean dtFollow;
        public final boolean brake;

        State(boolean spunUp, boolean dtFollow, boolean brake) {
            this.spunUp = spunUp;
            this.dtFollow = dtFollow;
            this.brake = brake;
        }
    }

    public enum Transfer {
        LEFT_BUMPER_START,
        RIGHT_BUMPER_START,
        LEFT_BUMPER_END,
        RIGHT_BUMPER_END
    }

    public long LL_INTERVAL_MS = 100;
    public double stddevPosMult = 35;
    public double stddevYawMult = 2;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Latch latch;
    public final Launcher launcher;
    public final PinpointLocalizer localizer;
    public final Limelight3A limelight;

    private State state = State.IDLE;
    private final ArrayDeque<State> futureStates = new ArrayDeque<>();
    private final boolean updateLocalizer;
    // TODO this only does things on rpm control for now
    // rememer to integrate when fix the drivetrain update weirdness
    private boolean stopAutoRpm = false;
    private boolean slowShoot = false;
    private boolean trustNextLimelight = false;

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

    public Robot(HardwareMap hardwareMap, PinpointLocalizer existingLocalizer, boolean updateLocalizer) {
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
                        // task = Task.newWithOneshot(() -> launcher.setTargetRpm(launcher.fallbackRpm));
                        task = Task.empty();
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
                        task = Task.newWithOneshot(() -> launcher.setFlywheelOn(false));
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
                        launcher.setFlywheelOn(false);
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

    private double moveCloserRot(double start, double end, double proportion) {
        return AngleUnit.normalizeRadians(start + AngleUnit.normalizeRadians(end - start) * proportion);
    }

    private long lastLLUpdate = 0;

    public void update() {
        long time = System.currentTimeMillis();

        launcher.update();
        if (updateLocalizer) {
            localizer.update();
        }
        double heading = localizer.getPose().heading.toDouble();
        drivetrain.update(heading);

        double xMm = localizer.getPose().position.x * 25.4;
        double yMm = localizer.getPose().position.y * 25.4;

        if (localizer.getPoseVel().linearVel.norm() < 0.5 && localizer.getPoseVel().angVel < 0.01) {
            LLResult result = limelight.getLatestResult();
            if (trustNextLimelight && (time - lastLLUpdate > LL_INTERVAL_MS && result != null && result.isValid())) {
                trustNextLimelight = false;
                lastLLUpdate = time;
                double[] stddev = result.getStddevMt1();
                double stddevX = stddev[0];
                double stddevY = stddev[1];
                double stddevYaw = stddev[5];
                Pose3D botpose = result.getBotpose();
                if (botpose.getPosition().x != 0 || botpose.getPosition().y != 0) {
                    /*
                    Pose2d newPos = new Pose2d(
                            moveCloser(xMm, botpose.getPosition().x * 1000,
                                    Math.exp(-stddevX * stddevPosMult)) / 25.4,
                            moveCloser(yMm, botpose.getPosition().y * 1000,
                                    Math.exp(-stddevY * stddevPosMult)) / 25.4,
                            // heading
                            moveCloserRot(heading, botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                                    Math.exp(-stddevYaw * stddevYawMult))
                    );
                     */
                    Pose2d newPos = new Pose2d(
                            botpose.getPosition().x * 1000 / 25.4,
                            botpose.getPosition().y * 1000 / 25.4,
                            botpose.getOrientation().getYaw(AngleUnit.RADIANS)
                    );
                    localizer.setPose(newPos);
                    // System.out.printf("x %6.2f %6.2f y %6.2f %6.2f rot %6.2f %6.2f\n", xMm / 25.4, botpose.getPosition().x * 1000 / 25.4, yMm / 25.4, botpose.getPosition().y * 1000 / 25.4, Math.toDegrees(heading), botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                }
            }
        }

        launcher.setFlywheelOn(state.spunUp && getNextState().map(s -> s.spunUp).orElse(true));
        // TODO hack
        // turn off flywheel, set, turn back on
        if (stopAutoRpm && launcher.isFlywheelOn()) {
            launcher.setFlywheelOn(false);
            launcher.setLauncherParams(localizer.getPose(), localizer.getPoseVel(), localizer.getPoseAccel());
            // overrided calculated rpm
            launcher.setTargetRpm(Launcher.fallbackRpm);
            launcher.setFlywheelOn(true);
        } else {
            launcher.setLauncherParams(localizer.getPose(), localizer.getPoseVel(), localizer.getPoseAccel());
        }

        drivetrain.setBrake(state.brake);
    }

    public void resetPos() {
        if (Color.getCurrentColor().orElse(Color.RED) == Color.RED) {
            // localizer.setPose(new Pose2d(new Vector2d(63.2, 61.1), localizer.getPose().heading));
            localizer.setPose(new Pose2d(new Vector2d(62.2, -63.4), Math.PI / 2));
        } else {
            localizer.setPose(new Pose2d(new Vector2d(62.2, 63.4), -Math.PI / 2));
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

    public void disableAutoRpm() {
        stopAutoRpm = true;
    }

    public void enableAutoRpm() {
        stopAutoRpm = false;
    }

    public boolean isAutoRpmStopped() {
        return stopAutoRpm;
    }

    public void readLimelight() {
        trustNextLimelight = true;
    }

    public void setSlowShoot(boolean slowShoot) {
        this.slowShoot = slowShoot;
    }

    public boolean isSlowShoot() {
        return slowShoot;
    }
}
