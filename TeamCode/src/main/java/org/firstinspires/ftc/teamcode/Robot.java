package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Map;

public class Robot {
    public enum State {
        INTAKING,
        IDLE,
        SHOOTING,
    }

    private final Intake intake;
    private final Latch latch;
    private final Launcher launcher;

    private State state = State.IDLE;

    public Robot(HardwareMap hardwareMap) {
        // ??
        VoltageSensor voltageSensor = new CachingVoltageSensor(hardwareMap.voltageSensor.iterator().next());
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        launcher = new Launcher(hardwareMap, voltageSensor);
    }

    public Map<State, Task> getTransitions() {
        switch (state) {
            case INTAKING:
                return Map.of(
                        State.IDLE, Task.empty().withOneshot(() -> intake.setPower(Intake.INTAKE_OFF))
                );
            case IDLE:
                return Map.of(
                        State.INTAKING, Task.empty().withOneshot(() -> intake.setPower(Intake.INTAKE_ON)),
                        State.SHOOTING, launcher.waitForSpinUp()
                                .andThen((Task.empty().withOneshot(() -> intake.setPower(Intake.INTAKE_TRANSFER))))
                                .andThen(latch.open())
                );
            case SHOOTING:
                return Map.of(
                        State.IDLE, Task.empty().withOneshot(() -> intake.setPower(Intake.INTAKE_OFF))
                                .with(latch.close())
                );
            default:
                throw new IllegalStateException("unknown state " + state);
        }
    }
}
