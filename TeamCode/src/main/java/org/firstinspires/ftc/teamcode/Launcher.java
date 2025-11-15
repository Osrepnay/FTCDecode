package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Arrays;

@Config
public class Launcher {
    public static final double RPM_TOLERANCE = 10;
    // public static final long RPM_SAMPLING_MS = 100;

    private final DcMotorEx[] motors;
    private final VoltageSensor voltageSensor;

    // really should be private final but dashboard
    public static PIDController pid = new PIDController(0.004, 0.000001, 0)
            .withIntegralRange(30);
    public static final Lerp rpmFeedforward = new Lerp(
            // 1620rpm
            /*
            new double[]{0, 242, 448, 647, 861, 1030, 1198, 1363},
            new double[]{0, .2368, .3552, .4736, .5920, .7104, .8288, .9472}
             */
            new double[]{0, 102, 219, 306, 420, 510, 622, 708, 819, 930, 1018, 1125, 1201, 1321, 1433, 1514},
            new double[]{0, .122, .183, .244, .305, .366, .427, .488, .549, .610, .671, .732, .793, .854, .915, .976}
    );
    public static final Lerp rpmDist = new Lerp(
            new double[][]{
                    {2990, 1095},
                    {2640, 1045},
                    {2570, 1010},
                    {2100, 970},
                    {1900, 960},
                    {1698, 885},
                    {1510, 868},
                    {1300, 850},
                    {850, 835},
            }
    );

    public double fallbackRpm = 1000;
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
