package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Objects;

@Config
public class Launcher {
    public static double LAUNCH_RPM = 1000;
    public static final double RPM_TOLERANCE = 10;
    // public static final long RPM_SAMPLING_MS = 100;

    private final DcMotorEx[] motors;
    private final VoltageSensor voltageSensor;

    // really should be private final but dashboard
    public static PIDController pid = new PIDController(0.004, 0.000001, 0)
            .withIntegralRange(30);
    public static final Lerp rpmFeedforward = new Lerp(
            // 1620rpm
            new double[]{0, 242, 448, 647, 861, 1030, 1198, 1363},
            new double[]{0, .2368, .3552, .4736, .5920, .7104, .8288, .9472}
            /*
            new double[]{0, 97, 227, 327, 444, 538, 654, 747, 861, 982, 1083, 1194, 1284, 1403, 1513, 1615},
            new double[]{0, 0.1293, 0.1939, 0.2586, 0.3232, 0.3879, 0.4525, 0.5172, 0.5818, 0.6464, 0.7111, 0.7757,
                    0.8404, 0.9050, 0.9697, 1.0343}
             */
            // 1150rpm
            /*
            new double[]{0, 98, 248, 392, 547, 701, 845, 983, 1124},
            new double[]{0, 0.1244, 0.2488, 0.3731, 0.4975, 0.6219, 0.7463, 0.8706, 0.9950}
             */
    );
    public static final Lerp rpmDist = new Lerp(
            new double[][]{
                    {3072, 1030},
                    {2634, 970},
                    {2260, 920},
            }
    );

    public double targetRpm = 0;

    /*
    // synced
    private final ArrayDeque<Long> tickTimes = new ArrayDeque<>();
    private final ArrayDeque<Long> tickCounts = new ArrayDeque<>();
    private double lastPower = 0;
     */
    private double currentRpm = 0;

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

    private boolean powerKilled = true;

    public void killPower() {
        powerKilled = true;
        Arrays.stream(motors).forEach(m -> m.setPower(0));
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(double rpm) {
        powerKilled = false;
        targetRpm = rpm;
    }

    public void setTargetRpmByDistance(double distMm) {
        setTargetRpm(rpmDist.interpolate(distMm));
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public boolean isAtTargetRpm() {
        return Math.abs(getCurrentRpm() - targetRpm) < RPM_TOLERANCE;
    }

    public Task waitForSpinUp() {
        return Task.empty().withUpdate(this::isAtTargetRpm);
    }

    public double update() {
        if (powerKilled) {
            return 0;
        }

        /*
        long time = System.currentTimeMillis();
        long currTicks = motors[1].getCurrentPosition();
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
        long totalTime = tickTimes.getLast() - tickTimes.getFirst();
        if (totalTime == 0) totalTime = 1;
        double tps = (double) totalTicks / totalTime * 1000;
        double rpm = tps / 103.8 * 60;
        currentRpm = rpm;
         */
        currentRpm = motors[1].getVelocity() / 103.8 * 60;

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
