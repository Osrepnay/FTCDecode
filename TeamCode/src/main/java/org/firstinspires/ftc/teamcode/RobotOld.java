package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.noncents.CachingIMU;
import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;
import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.TwoDeadWheelLocalizer;

import java.util.ArrayDeque;
import java.util.Optional;

@Config
public class RobotOld {
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

    public final CachingIMU imu;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Latch latch;
    public final Launcher launcher;
    public final Camera camera;
    public final Localizer localizer;
    public final Encoder par, perp;

    private State state = State.IDLE;
    private final ArrayDeque<State> futureStates = new ArrayDeque<>();
    private boolean forceUnlock = false;
    // TODO this only does things on rpm control for now
    // rememer to integrate when fix the drivetrain update weirdness
    private boolean cameraDisabled = false;
    private boolean slowShoot = false;

    public RobotOld(HardwareMap hardwareMap) {
        VoltageSensor voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        imu = new CachingIMU(hardwareMap.get(IMU.class, "imu"));
        drivetrain = new Drivetrain(hardwareMap, voltageSensor);
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        launcher = new Launcher(hardwareMap, voltageSensor);
        camera = new Camera(hardwareMap);
        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, MecanumDrive.PARAMS.inPerTick, new Pose2d(0, 0, 0));
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "wheelBackLeft")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "wheelFrontRight")));
        par.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Task init() {
        return latch.doClose();
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
                if (transfer == Transfer.RIGHT_BUMPER_START) {
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
                                .andThen(Task.newWithOneshot(() -> intake.setPower(Intake.INTAKE_BARELYMOVE)))
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

    public static double lateralMult = 1.99e-5;
    public static double axialMult = 2.99e-5;

    private long cameraLastUpdate = -1;

    public void update() {
        launcher.update();
        camera.update();
        imu.update();

        double rotationRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        double axialTps = Optional.ofNullable(par.getPositionAndVelocity().rawVelocity).orElse(0)
                - rotationRate * TwoDeadWheelLocalizer.PARAMS.parYTicks;
        /*
        double lateralTps = Optional.ofNullable(perp.getPositionAndVelocity().rawVelocity).orElse(0)
                - rotationRate * TwoDeadWheelLocalizer.PARAMS.perpXTicks;
         */
        double lateralTps = 0;

        /*
        boolean shouldBeLocked = false;
        boolean futureSpunUp = getNextState().map(s -> s.spunUp).orElse(false);
        if (!cameraDisabled && (state.spunUp || futureSpunUp)) {
            Optional<Double> camHeading = camera.getBearing();
            Optional<Double> camRange = camera.getRange();
            if (!forceUnlock) {
                shouldBeLocked = true;
                // make sure we're not putting stale camera angles into drivetrain
                // because lockHeading is relative
                if (cameraLastUpdate != camera.lastUpdate()) {
                    cameraLastUpdate = camera.lastUpdate();
                    Optional<Double> adjustedHeading = camHeading.flatMap(h -> camRange.map(r -> {
                        double x = Math.sin(h) * r;
                        x -= lateralTps * r * lateralMult;

                        double y = Math.cos(h) * r;
                        y -= axialTps * r * axialMult;

                        return Math.atan2(x, y);
                    }));
                    if (adjustedHeading.isPresent()) {
                        drivetrain.lockHeading(camHeading.get());
                        // TODO disabled because it screws with the control system too much
                        double speedAdjClamp = 0.00;
                        drivetrain.setHeadingBias(Math.min(speedAdjClamp, Math.max(-speedAdjClamp,
                                adjustedHeading.get() - camHeading.get())));
                    } else {
                        // drivetrain.unlockHeading();
                    }
                }
            }
            if (futureSpunUp || !getNextState().isPresent()) {
                camera.getRange().ifPresent(r -> {
                    // TODO use proper range instead of hypot range
                    // r -= axialTps * r * axialMult;
                    launcher.setTargetRpmByDistance(r);
                });
            }
        }
        if (!shouldBeLocked) {
            drivetrain.unlockHeading();
        }
         */
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

    public void enableCamera() {
        cameraDisabled = false;
    }

    public void disableCamera() {
        cameraDisabled = true;
    }

    public boolean isCameraDisabled() {
        return cameraDisabled;
    }

    public void setSlowShoot(boolean slowShoot) {
        this.slowShoot = slowShoot;
    }

    public boolean isSlowShoot() {
        return slowShoot;
    }
}
