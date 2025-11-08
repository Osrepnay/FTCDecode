package org.firstinspires.ftc.teamcode.noncents;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class CachingIMU implements IMU {
    private final IMU imu;

    public CachingIMU(IMU imu) {
        this.imu = imu;
    }

    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return imu.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return imu.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        imu.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        imu.close();
    }

    @Override
    public boolean initialize(Parameters parameters) {
        return imu.initialize(parameters);
    }

    @Override
    public void resetYaw() {
        imu.resetYaw();
    }

    private YawPitchRollAngles lastYPRA = null;
    private Orientation lastOrientation = null;
    private Quaternion lastQuaternion = null;
    private AngularVelocity lastAngVel = null;

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        if (lastYPRA == null) {
            lastYPRA = imu.getRobotYawPitchRollAngles();
        }
        return lastYPRA;
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        if (lastOrientation == null) {
            lastOrientation = imu.getRobotOrientation(reference, order, angleUnit);
        }
        return lastOrientation.toAxesReference(reference).toAxesOrder(order).toAngleUnit(angleUnit);
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        if (lastQuaternion == null) {
            lastQuaternion = imu.getRobotOrientationAsQuaternion();
        }
        return lastQuaternion;
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        if (lastAngVel == null) {
            lastAngVel = imu.getRobotAngularVelocity(angleUnit);
        }
        return lastAngVel.toAngleUnit(angleUnit);
    }

    public void update() {
        lastYPRA = null;
        lastOrientation = null;
        lastQuaternion = null;
        lastAngVel = null;
    }
}
