package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;

import java.util.Optional;

// import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Drivetrain {
    public final DcMotorEx[] wheels;
    public final IMU imu;
    public final VoltageSensor voltageSensor;
    // normal pid: 5, 0, 60
    // TODO static is suuper bad, but needed for dashboard
    public static PIDController headingPid = new PIDController(0.7, 0.000, 0)
            .withIntegralRange(5)
            .withIntegralCap(200);
    public static double lowpassOldFrac = 0.82;

    public Drivetrain(DcMotorEx[] wheels, IMU imu, VoltageSensor voltageSensor) {
        for (int i = 0; i < wheels.length; i++) {
            // wheels[i] = new CachingDcMotorEx(wheels[i]);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        this.wheels = wheels;

        this.imu = imu;
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        this.imu.resetYaw();

        this.voltageSensor = voltageSensor;
    }

    public Drivetrain(HardwareMap hardwareMap, IMU imu, VoltageSensor voltageSensor) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        }, imu, voltageSensor);
    }

    private double currentHeading = 0;

    public double getHeading() {
        return currentHeading;
    }

    private boolean headingLocked = false;
    private boolean headingLockSet = false;
    private double headingLock = 0;
    private double headingBias = 0;
    private double rawHeading = 0;

    public double getRawHeading() {
        return rawHeading + headingBias;
    }

    public void setHeadingBias(double rad) {
        headingBias = rad;
    }

    public double getHeadingBias() {
        return headingBias;
    }

    // normalizes to (-180, 180]
    private double normalizeHeading(double heading) {
        heading %= Math.PI * 2;
        // there's probably a more elegant way but brian on work
        if (heading <= -Math.PI) {
            heading += Math.PI * 2;
        } else if (heading > Math.PI) {
            heading -= Math.PI * 2;
        }
        return heading;
    }

    // lock a heading relative to the current one
    public void lockHeading(double headingRad) {
        headingLocked = true;
        double heading = getHeading();
        double newHeading;
        rawHeading = normalizeHeading(heading + headingRad);
        // disable lowpass math if no previous data
        if (!headingLockSet) {
            newHeading = rawHeading;
        } else {
            double diff = normalizeHeading(rawHeading - headingLock);
            newHeading = normalizeHeading(headingLock + diff * (1 - lowpassOldFrac));
        }
        headingLockSet = true;
        headingLock = newHeading;
    }

    public void unlockHeading() {
        headingLocked = false;
        headingLockSet = false;
        headingLock = 0;
        headingBias = 0;
    }

    public boolean isHeadingLocked() {
        return headingLocked;
    }

    public Optional<Double> getHeadingLock() {
        if (headingLocked) {
            return Optional.of(headingLock + headingBias);
        } else {
            return Optional.empty();
        }
    }

    public void driveFieldCentric(double forward, double lateral, double rotate) {
        double yaw = getHeading();
        driveBotCentric(
                Math.cos(yaw) * forward - Math.sin(yaw) * lateral,
                Math.sin(yaw) * forward + Math.cos(yaw) * lateral,
                rotate
        );
    }

    public double fullSqrt(double x) {
        if (x < 0) {
            return -Math.sqrt(-x);
        } else {
            return Math.sqrt(x);
        }
    }

    public void driveBotCentric(double forward, double lateral, double rotate) {
        currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        Lerp orthoLerp = new Lerp(new double[][]{
                {0, 0},
                {0.02, 0.03},
                {0.5, 0.35},
                {1, 1}
        });
        Lerp rotateLerp = new Lerp(new double[][]{
                {0, 0},
                {0.02, 0.02},
                {0.4, 0.15},
                {1, 1}
        });

        forward = orthoLerp.interpolateMagnitude(forward);
        lateral = orthoLerp.interpolateMagnitude(lateral);
        Optional<Double> headingLock = getHeadingLock();
        rotate = rotateLerp.interpolateMagnitude(rotate);
        if (headingLock.isPresent()) {
            // center at headingLock, easier math
            double headingErr = normalizeHeading(getHeading() - headingLock.get());
            rotate += -headingPid.update(0, fullSqrt(headingErr)) * 13 / voltageSensor.getVoltage();
            // rotate = -headingPid.update(0, headingErr) * 13 / voltageSensor.getVoltage();
        }
        double[] powers = {
                forward + lateral + rotate,
                forward - lateral - rotate,
                forward - lateral + rotate,
                forward + lateral - rotate
        };
        if (isHeadingLocked()) {
            double max = 1;
            for (int i = 0; i < wheels.length; i++) {
                max = Math.max(Math.abs(powers[i]), max);
            }
            double space = 1 - Math.abs(rotate);
            double extra = max - 1;
            if (extra > 0) {
                double fac = space / (space + extra);
                forward *= fac;
                lateral *= fac;
                powers = new double[]{
                        forward + lateral + rotate,
                        forward - lateral - rotate,
                        forward - lateral + rotate,
                        forward + lateral - rotate
                };
            }
        }
        setRawPowers(powers);
    }

    public void setRawPowers(double[] powers) {
        // this helps maintain heading when oversaturated
        double max = 1;
        for (int i = 0; i < wheels.length; i++) {
            max = Math.max(Math.abs(powers[i]), max);
        }
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(powers[i] / max);
        }
    }
}
