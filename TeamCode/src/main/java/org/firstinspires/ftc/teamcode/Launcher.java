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
            // double motor
            new double[]{0, 102, 219, 306, 420, 510, 622, 708, 819, 930, 1018, 1125, 1201, 1321, 1433, 1514},
            new double[]{0, .122, .183, .244, .305, .366, .427, .488, .549, .610, .671, .732, .793, .854, .915, .976}
            /*
            new double[]{0, 105, 298, 661, 885},
            new double[]{0, 0.3, 0.4798, 0.78, 0.96}
             */
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

    public double fallbackRpm = 600;
    public double targetRpm = 0;

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
