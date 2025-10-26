package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;

import java.util.Optional;

// import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Drivetrain {
    private final DcMotorEx[] wheels;
    private final IMU imu;
    public static PIDController headingPid = new PIDController(0.7, 0, 0.7);

    public Drivetrain(DcMotorEx[] wheels, IMU imu) {
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
    }

    public Drivetrain(HardwareMap hardwareMap) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        }, hardwareMap.get(IMU.class, "imu"));
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private Optional<Double> headingLock = Optional.empty();

    // lock a heading relative to the current one
    public void lockHeading(double headingRad) {
        double heading = getHeading();
        double newHeading = heading + headingRad;
        headingLock = Optional.of(newHeading);
    }

    public void unlockHeading() {
        headingLock = Optional.empty();
    }

    public boolean isHeadingLocked() {
        return headingLock.isPresent();
    }

    public Optional<Double> getHeadingLock() {
        return headingLock;
    }

    public void driveFieldCentric(double forward, double lateral, double rotate) {
        double yaw = getHeading();
        driveBotCentric(
                Math.cos(yaw) * forward - Math.sin(yaw) * lateral,
                Math.sin(yaw) * forward + Math.cos(yaw) * lateral,
                rotate
        );
    }

    public void driveBotCentric(double forward, double lateral, double rotate) {
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
        ;
        forward = orthoLerp.interpolateMagnitude(forward);
        lateral = orthoLerp.interpolateMagnitude(lateral);
        if (headingLock.isPresent()) {
            // center at headingLock, easier math
            double heading = getHeading() - headingLock.get();
            // wraparound
            if (heading < -Math.PI) {
                heading += Math.PI * 2;
            } else if (heading > Math.PI) {
                heading -= Math.PI * 2;
            }
            rotate = -headingPid.update(0, heading);
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
