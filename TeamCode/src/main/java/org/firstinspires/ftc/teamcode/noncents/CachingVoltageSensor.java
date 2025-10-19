package org.firstinspires.ftc.teamcode.noncents;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class CachingVoltageSensor implements VoltageSensor {
    private final VoltageSensor sensor;

    private double cachedVoltage = -1;
    private long lastPoll = -1;
    private long pollTimeMs = 250;

    public CachingVoltageSensor(VoltageSensor sensor) {
        this.sensor = sensor;
    }

    public double getVoltage() {
        long time = System.currentTimeMillis();
        if (cachedVoltage == -1 || time - lastPoll >= pollTimeMs) {
            cachedVoltage = sensor.getVoltage();
        }
        lastPoll = time;
        return cachedVoltage;
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
