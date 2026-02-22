package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.noncents.EMAFilter;
import org.firstinspires.ftc.teamcode.noncents.Lerp;
import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.tinfour.common.IIncrementalTin;
import org.tinfour.common.Vertex;
import org.tinfour.interpolation.IInterpolatorOverTin;
import org.tinfour.interpolation.NaturalNeighborInterpolator;
import org.tinfour.standard.IncrementalTin;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

@Config
public class Launcher {
    public static final double RPM_TOLERANCE = 30;
    // public static final long RPM_SAMPLING_MS = 100;

    public static final int TURRET_MIN_TICKS = -502;
    public static final int TURRET_MAX_TICKS = 570;
    public static final double TURRET_TICKS_PER_REV = 384.5 * 125 / 27;
    // public static double RPM_TO_IPS = Math.PI * 96 / 25.4 / 60 * 0.;
    public static double RPM_TO_IPS = 0.078;

    private final DcMotorEx[] motors;
    private final Hood hood;
    private final DcMotorEx turret;
    private final VoltageSensor voltageSensor;

    // really should be private final but dashboard
    public static PIDController pid = new PIDController(0.005, 0.000001, 0)
            .withIntegralRange(30);
    public static final Lerp rpmFeedforward = new Lerp(
            new double[] {0, 596, 597, 1023, 1499, 1883, 2304, 2665, 3039, 3487, 3822, 4144, 4413, 4710, 4980, 5199},
            new double[] {0, 0, .18, .24, .31, .37, .43, .49, .55, .61, .67, .73, .79, .86, .92, .98}
    );
    public static EMAFilter rpmFilter = new EMAFilter(0.6);

    private final double[][] launcherParamsByPos = {
            // x, y, rpm, hood, offset from center (positive is horz, negative is vert)
            // tuned from red perspective
            // @formatter:off
            // near
            { -33,   54, 2130, 0.0,  -12},
            { -63,   26, 2130, 0.0,   15},
            { -50,   -4, 2400, 0.8,    6},
            { -64,  -44, 2730, 0.8,   15},
            {  14,    8, 2650, 0.8,   -8},
            { -17,   23, 2320, 0.7,   -8},

            // far
            {  49,   10, 3050, 1.0,   -7},
            {  64,  -21, 3340, 1.0,   -6},
            {  65,   28, 3080, 0.9,   -9},
            {  65,  -55, 3530, 0.9,   -9},
            {  65,  -56, 2980, 1.0,  -10},
            // @formatter:on
    };

    private final IInterpolatorOverTin rpmInterp;
    private final IInterpolatorOverTin hoodInterp;
    private final IInterpolatorOverTin offsetInterp;

    public static boolean disableInterp = false;
    public static double fallbackRpm = 2400;
    public static double fallbackHood = 1;
    public static double fallbackOffset = 0;
    private boolean disableVelCorr = false;
    private double targetRpm = 0;
    private double currentRpm = 0;
    private boolean flywheelOn = false;
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
        PIDFCoefficients turretCoeff = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        turretCoeff.p *= 2.2;
        turretCoeff.i *= 0.6;
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, turretCoeff);
        // turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.voltageSensor = voltageSensor;

        Function<Integer, IInterpolatorOverTin> mkInterp = idx -> {
            List<Vertex> vertices = Arrays.stream(launcherParamsByPos)
                    .map(xs -> new Vertex(xs[0], xs[1], xs[idx]))
                    .collect(Collectors.toList());
            IIncrementalTin tin = new IncrementalTin();
            tin.add(vertices, null);
            return new NaturalNeighborInterpolator(tin);
        };
        rpmInterp = mkInterp.apply(2);
        hoodInterp = mkInterp.apply(3);
        offsetInterp = mkInterp.apply(4);
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

    private Vector2d applyGoalOffset(Vector2d vec, double offset) {
        if (offset < 0) {
            return new Vector2d(vec.x, vec.y + offset);
        } else {
            return new Vector2d(vec.x + offset, vec.y);
        }
    }

    public void setLauncherParams(Pose2d robotPose, PoseVelocity2d poseVel) {
        Pose2d launcherPose = robotPose.plus(new Twist2d(new Vector2d( -22.8 / 25.4, 0), 0));

        Vector2d rVec = launcherPose.position;
        double rHeading = launcherPose.heading.toDouble();
        Vector2d rVelVec = new Vector2d(
                Math.cos(rHeading) * poseVel.linearVel.x - Math.sin(rHeading) * poseVel.linearVel.y,
                Math.cos(rHeading) * poseVel.linearVel.y + Math.sin(rHeading) * poseVel.linearVel.x
        );
        // flip if blue, tuned values are only from red
        // flip params back later
        double colorMult = Color.currentAsMult();
        rVec = new Vector2d(rVec.x, rVec.y * colorMult);
        rVelVec = new Vector2d(rVelVec.x, rVelVec.y * colorMult);
        rHeading *= colorMult;

        Vector2d goal = new Vector2d(-(70.75 - 2.5), 70.75 - 2.5);
        double targetRpm = disableInterp ? fallbackRpm : rpmInterp.interpolate(rVec.x, rVec.y, null);
        double targetHood = disableInterp ? fallbackHood : hoodInterp.interpolate(rVec.x, rVec.y, null);
        double targetOffset = disableInterp ? fallbackOffset : offsetInterp.interpolate(rVec.x, rVec.y, null);
        if (!disableInterp && !disableVelCorr && !Double.isNaN(targetRpm)) {
            Vector2d adjustedPos = rVec;
            for (int i = 0; i < 5; i++) {
                Vector2d newGoal = applyGoalOffset(goal, targetOffset);
                // inches per second
                double launchSpeed = RPM_TO_IPS * targetRpm * Math.sin(Math.toRadians(Hood.fracToDeg(targetHood)));
                double dist = newGoal.minus(adjustedPos).norm();
                double timeOfFlight = dist / launchSpeed;

                Vector2d lastAdjustedPos = adjustedPos;
                adjustedPos = rVec.plus(rVelVec.times(timeOfFlight));

                double newRpm = rpmInterp.interpolate(adjustedPos.x, adjustedPos.y, null);
                double newHood = hoodInterp.interpolate(adjustedPos.x, adjustedPos.y, null);
                double newOffset = offsetInterp.interpolate(adjustedPos.x, adjustedPos.y, null);
                if (Double.isNaN(newRpm) || adjustedPos.minus(lastAdjustedPos).norm() < 1) {
                    break;
                } else {
                    targetRpm = newRpm;
                    targetHood = newHood;
                    targetOffset = newOffset;
                }
            }
            rVec = adjustedPos;
        }
        if (Double.isNaN(targetRpm)) {
            targetRpm = fallbackRpm;
            targetHood = fallbackHood;
            targetOffset = fallbackOffset;
        }

        goal = applyGoalOffset(goal, targetOffset);

        double lockRad = Math.atan2(
                goal.y - rVec.y,
                goal.x - rVec.x
        ) - (rHeading + Math.PI);
        setTurretRadians(lockRad * colorMult);

        setTargetRpm(targetRpm);
        setHood(Math.min(1, Math.max(0, targetHood)));
    }

    public Task waitForSpinUp() {
        return Task.newWithUpdate(this::isAtTargetRpm);
    }

    public void setTurretRadians(double rad) {
        int ticks = (int) (AngleUnit.normalizeRadians(-rad) / 2 / Math.PI * TURRET_TICKS_PER_REV);
        if (ticks < TURRET_MIN_TICKS || ticks > TURRET_MAX_TICKS) {
            turret.setTargetPosition(Math.min(TURRET_MAX_TICKS, Math.max(TURRET_MIN_TICKS, ticks)));
            return;
        }
        turret.setTargetPosition(ticks);
        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(1);
        }
        currentRad = rad;
    }

    public void update() {
        if (flywheelOn) {
            currentRpm = rpmFilter.update(motors[0].getVelocity() * 3);

            double newPower = (rpmFeedforward.interpolateMagnitude(targetRpm) + pid.update(targetRpm, getCurrentRpm()))
                    * 13 / voltageSensor.getVoltage();
            for (DcMotorEx motor : motors) {
                motor.setPower(newPower);
            }
        } else {
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
        }
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

    // testing
    public void setRawPower(double power) {
        Arrays.stream(motors).forEach(m -> m.setPower(power));
    }

    public Task doSetHood(double pos) {
        return hood.doSetPos(pos);
    }

    public void setHood(double pos) {
        hood.setPos(pos);
    }

    public double getHoodPos() {
        return hood.getPos();
    }

    public double getTurretRadians() {
        return currentRad;
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public void setFlywheelOn(boolean shooterOn) {
        this.flywheelOn = shooterOn;
    }

    public void setVelCorrectionEnabled(boolean velCorr) {
        this.disableVelCorr = !velCorr;
    }
}