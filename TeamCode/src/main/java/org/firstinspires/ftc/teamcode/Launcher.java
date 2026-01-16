package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.noncents.EMAFilter;
import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Arrays;

@Config
public class Launcher {
    public static final double RPM_TOLERANCE = 30;
    // public static final long RPM_SAMPLING_MS = 100;

    public static final int TURRET_MIN_TICKS = -702;
    public static final int TURRET_MAX_TICKS = 800;
    public static final double TURRET_TICKS_PER_REV = 537.7 * 125 / 27;

    private final DcMotorEx[] motors;
    private final Hood hood;
    private final DcMotorEx turret;
    private final VoltageSensor voltageSensor;

    // really should be private final but dashboard
    public static PIDController pid = new PIDController(0.005, 0.000001, 0)
            .withIntegralRange(30);
    public static final Lerp rpmFeedforward = new Lerp(
            new double[]{0, 343, 838, 1212, 1683, 2065, 2512, 2865, 3345, 3832, 4195, 4612, 4945, 5332, 5776, 6078},
            new double[]{0, .1314, .1971, .2629, .3286, .3943, .4600, .5257, .5914, .6572, .7229, .7886, .8543, .9200, .9857, 1.0515}
    );
    public static final Lerp rpmDist = new Lerp(
            new double[][]{
                    {160 * 25.4, 3400},
                    {138 * 25.4, 3130},
                    {100 * 25.4, 2700},
                    {71 * 25.4, 2530},
            }
    );
    public static EMAFilter rpmFilter = new EMAFilter(0.3);

    public double fallbackRpm = 2400;
    private double targetRpm = 0;
    private double currentRpm = 0;
    // measured from front-on, counterclockwise positive, clockwise negative
    private double currentRad = 0;

    public Launcher(DcMotorEx[] motors, Hood hood, DcMotorEx turret, VoltageSensor voltageSensor) {
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.motors = motors;
        this.hood = hood;
        this.turret = turret;
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        this.voltageSensor = voltageSensor;
    }

    public Launcher(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this(
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class, "launcher0"),
                        hardwareMap.get(DcMotorEx.class, "launcher1"),
                },
                new Hood(hardwareMap),
                hardwareMap.get(DcMotorEx.class, "turret"),
                voltageSensor
        );
    }

    public Task doInit() {
        return hood.doSetPos(1);
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
        return Task.newWithUpdate(this::isAtTargetRpm);
    }

    // testing
    public void setRawPower(double power) {
        Arrays.stream(motors).forEach(m -> m.setPower(power));
    }

    public Task doSetHood(double pos) {
        return hood.doSetPos(pos);
    }

    public double getHoodPos() {
        return hood.getPos();
    }

    public boolean setTurretRadians(double rad) {
        int ticks = (int) (AngleUnit.normalizeRadians(-rad) / 2 / Math.PI * TURRET_TICKS_PER_REV);
        if (ticks < TURRET_MIN_TICKS || ticks > TURRET_MAX_TICKS) {
            return false;
        }
        turret.setTargetPosition(ticks);
        currentRad = rad;
        return true;
    }

    public double getTurretRadians() {
        return currentRad;
    }

    public void update() {
        if (powerKilled) {
            return;
        }

        currentRpm = rpmFilter.update(motors[0].getVelocity() * 3);

        double newPower = (rpmFeedforward.interpolateMagnitude(targetRpm) + pid.update(targetRpm, getCurrentRpm()))
                * 13 / voltageSensor.getVoltage();
        for (DcMotorEx motor : motors) {
            motor.setPower(newPower);
        }
    }
}