/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class Camera {
    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    public static final long POLL_MS = 1000 / 30;
    public static final double CAMERA_PITCH = Math.toRadians(13.5);

    public Camera(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
                .setCameraPose(
                        new Position(DistanceUnit.MM, 112, 0, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + CAMERA_PITCH, 0, 0)
                )
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setShowStatsOverlay(true)
                .enableLiveView(true)
                .setCameraResolution(new Size(960, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    private boolean isStreaming = false;
    private long last = -1;
    private boolean lastFound = false;
    private double lastBearing = 0;
    private double lastRange = 0;

    public void update() {
        long time = System.currentTimeMillis();
        if (time - last < POLL_MS) {
            return;
        }
        last = time;

        boolean wasStreaming = isStreaming;
        isStreaming = portal.getCameraState() == VisionPortal.CameraState.STREAMING;

        if (isStreaming && !wasStreaming) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            TimeUnit timeUnit = TimeUnit.MICROSECONDS;
            exposureControl.setExposure(1000, timeUnit);
            System.out.println(exposureControl.getMinExposure(TimeUnit.MICROSECONDS));

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(255);
        }

        List<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 24 || detection.id == 20) {
                double bearing = detection.ftcPose.bearing;
                double range = Math.cos(CAMERA_PITCH) * detection.ftcPose.range;
                double z = range * Math.cos(bearing);
                double realY = range * Math.sin(bearing) - 140; // 112 theoretical?
                double adjustedBearing = Math.atan2(realY, z);
                lastRange = Math.sqrt(z * z + realY * realY);
                lastBearing = adjustedBearing;
                lastFound = true;
                return;
            }
        }

        lastFound = false;
    }

    public Optional<Double> getRange() {
        if (lastFound) {
            return Optional.of(lastRange);
        } else {
            return Optional.empty();
        }
    }

    public Optional<Double> getBearing() {
        if (lastFound) {
            return Optional.of(lastBearing);
        } else {
            return Optional.empty();
        }
    }

    public long lastUpdate() {
        return last;
    }
}