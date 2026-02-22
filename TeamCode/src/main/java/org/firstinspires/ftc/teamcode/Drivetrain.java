package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;

import java.util.Optional;

// import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Drivetrain {
    public static double LATERAL_MULT = 1.4;
    public static PIDController headingPid = new PIDController(1, 0, 0);

    public final DcMotorEx[] wheels;
    public final VoltageSensor voltageSensor;

    public Drivetrain(DcMotorEx[] wheels, VoltageSensor voltageSensor) {
        for (int i = 0; i < wheels.length; i++) {
            // wheels[i] = new CachingDcMotorEx(wheels[i]);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        this.wheels = wheels;
        this.voltageSensor = voltageSensor;
    }

    public Drivetrain(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        }, voltageSensor);
    }

    private double currentHeading = 0;

    public void update(double heading) {
        currentHeading = AngleUnit.normalizeRadians(heading);
    }

    public double getHeading() {
        return currentHeading;
    }

    private final Lerp orthoLerp = new Lerp(new double[][]{
            {0, 0},
            {0.02, 0.03},
            {0.5, 0.35},
            {1, 1}
    });
    private final Lerp rotateLerp = new Lerp(new double[][]{
            {0, 0},
            {0.02, 0.02},
            {0.4, 0.15},
            {1, 1}
    });

    public void driveFieldCentricFollow(double forward, double lateral, double rotate) {
        double yaw = getHeading();
        if (Color.getCurrentColor().equals(Optional.of(Color.BLUE))) {
            forward = -forward;
            lateral = -lateral;
        }
        double totalDrive = Math.hypot(forward, lateral);
        double controlHeading = Math.atan2(forward, lateral);
        double normErr = AngleUnit.normalizeRadians(controlHeading - yaw);
        // allow driving backwards too, as long as it's orthogonal
        if (normErr > Math.PI / 2) {
            normErr = normErr - Math.PI;
        } else if (normErr < -Math.PI / 2) {
            normErr = normErr + Math.PI;
        }
        // it's not the true error anymore but this is to simulate SQUID
        double fakeErr = fullSqrt(normErr);
        controlHeading = yaw + fakeErr;
        if (totalDrive > 0.9 && Math.abs(normErr) < Math.toRadians(45)) {
            rotate -= headingPid.update(controlHeading, yaw);
        }
        driveFieldCentric(forward, lateral, rotate);
    }

    public void driveFieldCentric(double forward, double lateral, double rotate) {
        double yaw = getHeading();
        forward = orthoLerp.interpolateMagnitude(forward);
        lateral = orthoLerp.interpolateMagnitude(lateral);
        rotate = rotateLerp.interpolateMagnitude(rotate);
        if (Color.getCurrentColor().equals(Optional.of(Color.BLUE))) {
            forward = -forward;
            lateral = -lateral;
        }
        driveRaw(
                Math.sin(yaw) * forward + Math.cos(yaw) * lateral,
                -Math.cos(yaw) * forward + Math.sin(yaw) * lateral,
                rotate
        );
    }

    private double fullSqrt(double x) {
        if (x < 0) {
            return -Math.sqrt(-x);
        } else {
            return Math.sqrt(x);
        }
    }

    public void driveBotCentric(double forward, double lateral, double rotate) {
        forward = orthoLerp.interpolateMagnitude(forward);
        lateral = orthoLerp.interpolateMagnitude(lateral);
        rotate = rotateLerp.interpolateMagnitude(rotate);

        driveRaw(forward, lateral, rotate);
    }

    private void driveRaw(double forward, double lateral, double rotate) {
        lateral *= LATERAL_MULT;
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

    public void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior mode = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setZeroPowerBehavior(mode);
        }
    }
}
