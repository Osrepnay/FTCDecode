package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Objects;

@Config
public class Launcher {
    public static final double LAUNCH_RPM = 1100;
    public static final double RPM_TOLERANCE = 10;
    public static final long RPM_SAMPLING_MS = 75;

    private final DcMotorEx[] motors;
    private final VoltageSensor voltageSensor;

    // really should be private final but dashboard
    public static PIDController pid = new PIDController(0.003, 0, 0.04)
            .withIntegralRange(100);
    public static final Lerp rpmFeedforward = new Lerp(
            new double[]{0, 97, 227, 327, 444, 538, 654, 747, 861, 982, 1083, 1194, 1284, 1403, 1513, 1615},
            new double[]{0, 0.1293, 0.1939, 0.2586, 0.3232, 0.3879, 0.4525, 0.5172, 0.5818, 0.6464, 0.7111, 0.7757,
                    0.8404, 0.9050, 0.9697, 1.0343}
    );

    public double targetRpm = 0;

    // synched
    private final ArrayDeque<Long> tickTimes = new ArrayDeque<>();
    private final ArrayDeque<Long> tickCounts = new ArrayDeque<>();
    private double lastRpm = 0;
    private double currentRpm = 0;
    private final double lowpassPrevFac = 0.9;

    public Launcher(DcMotorEx[] motors, VoltageSensor voltageSensor) {
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.motors = motors;
        this.voltageSensor = voltageSensor;
    }

    public Launcher(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "launcher0"),
                hardwareMap.get(DcMotorEx.class, "launcher1"),
        }, voltageSensor);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public boolean isAtTargetRpm() {
        return Math.abs(getCurrentRpm() - targetRpm) < RPM_TOLERANCE;
    }

    public Task waitForSpinUp() {
        setTargetRpm(LAUNCH_RPM);
        return Task.empty().withUpdate(this::isAtTargetRpm);
    }

    public double update() {
        long time = System.currentTimeMillis();
        long currTicks = motors[0].getCurrentPosition();
        long deltaMs = tickTimes.isEmpty() ? Long.MAX_VALUE : time - tickTimes.getLast();

        tickTimes.offerLast(time);
        tickCounts.offerLast(currTicks);

        long minimumMs = time - RPM_SAMPLING_MS;
        // silence the warning, should really never be null anyways
        while (Objects.requireNonNull(tickTimes.getFirst()) < minimumMs) {
            tickTimes.removeFirst();
            tickCounts.removeFirst();
        }

        long totalTicks = tickCounts.getLast() - tickCounts.getFirst();
        double tps = (double) totalTicks / RPM_SAMPLING_MS * 1000;
        double rpm = tps / 103.8 * 60;
        lastRpm = currentRpm;
        // TODO so sketchy
        double prevFacAdjusted = lowpassPrevFac * Math.exp(-deltaMs / 50.0);
        currentRpm = rpm * (1 - prevFacAdjusted) + lastRpm * prevFacAdjusted;

        double newPower = (rpmFeedforward.interpolateMagnitude(targetRpm) + pid.update(targetRpm, getCurrentRpm()))
                * 13 / voltageSensor.getVoltage();
        for (DcMotorEx motor : motors) {
            motor.setPower(newPower);
        }
        return newPower;
    }

    // testing
    public void setRawPower(double power) {
        Arrays.stream(motors).forEach(m -> m.setPower(power));
    }
}
