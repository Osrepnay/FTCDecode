package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.noncents.Lerp;

// import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Drivetrain {
    private final DcMotorEx[] wheels;

    public Drivetrain(DcMotorEx[] wheels) {
        for (int i = 0; i < wheels.length; i++) {
            // wheels[i] = new CachingDcMotorEx(wheels[i]);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[3].setDirection(DcMotor.Direction.REVERSE);
        this.wheels = wheels;
    }

    public Drivetrain(HardwareMap hardwareMap) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        });
    }

    public double[] setMixedPowers(double forward, double lateral, double rotate) {
        Lerp forwardLerp = new Lerp(new double[][] {
                {0, 0},
                {0.02, 0.03},
                {0.5, 0.35},
                {1, 1}
        });
        Lerp lateralLerp = forwardLerp;
        Lerp rotateLerp = new Lerp(new double[][] {
                {0, 0},
                {0.02, 0.02},
                {0.4, 0.15},
                {1, 1}
        });;
        forward = forwardLerp.interpolateMagnitude(forward);
        lateral = lateralLerp.interpolateMagnitude(lateral);
        rotate = rotateLerp.interpolateMagnitude(rotate);
        double[] powers = {
                forward + lateral + rotate,
                forward - lateral - rotate,
                forward - lateral + rotate,
                forward + lateral - rotate
        };
        setRawPowers(powers);
        return powers;
    }

    public void setRawPowers(double[] powers) {
        double max = 1;
        for (int i = 0; i < wheels.length; i++) {
            max = Math.max(Math.abs(powers[i]), max);
        }
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(powers[i] / max);
        }
    }
}
