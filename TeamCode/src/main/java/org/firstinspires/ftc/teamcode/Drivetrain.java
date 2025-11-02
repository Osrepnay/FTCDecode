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
    private final DcMotorEx[] wheels;
    private final IMU imu;
    private final VoltageSensor voltageSensor;
    // normal pid: 5, 0, 60
    public static PIDController headingPid = new PIDController(0.9, 0.000, 2)
            .withIntegralRange(5)
            .withIntegralCap(200);

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

    public Drivetrain(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        }, hardwareMap.get(IMU.class, "imu"), voltageSensor);
    }

    private double currentHeading = 0;

    public double getHeading() {
        return currentHeading;
    }

    private boolean headingLocked = false;
    private boolean headingLockSet = false;
    private double headingLock = 0;
    public static double lowpassOldFrac = 0.9;

    // lock a heading relative to the current one
    public void lockHeading(double headingRad) {
        headingLocked = true;
        double heading = getHeading();
        double newHeading;
        // disable lowpass math if no previous data
        if (!headingLockSet) {
            newHeading = heading + headingRad;
        } else {
            newHeading = headingLock * lowpassOldFrac + (heading + headingRad) * (1 - lowpassOldFrac);
        }
        headingLockSet = true;
        headingLock = newHeading;
    }

    public void unlockHeading() {
        headingLocked = false;
        headingLockSet = false;
        headingLock = 0;
    }

    public boolean isHeadingLocked() {
        return headingLocked;
    }

    public Optional<Double> getHeadingLock() {
        if (headingLocked) {
            return Optional.of(headingLock);
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
        if (isHeadingLocked()) {
            // center at headingLock, easier math
            double heading = getHeading() - headingLock;
            // wraparound
            if (heading < -Math.PI) {
                heading += Math.PI * 2;
            } else if (heading > Math.PI) {
                heading -= Math.PI * 2;
            }
            rotate = -headingPid.update(0, fullSqrt(heading)) * 13 / voltageSensor.getVoltage();
        } else {
            rotate = rotateLerp.interpolateMagnitude(rotate);
        }
        double[] powers = {
                forward + lateral + rotate,
                forward - lateral - rotate,
                forward - lateral + rotate,
                forward + lateral - rotate
        };
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
