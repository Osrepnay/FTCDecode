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

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class Camara extends OpMode {
    private Telemetry dash;
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private InputManager inputManager;

    @Override
    public void init() {
        dash = FtcDashboard.getInstance().getTelemetry();
        inputManager = new InputManager();
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setShowStatsOverlay(true)
                .enableLiveView(true)
                .build();
    }

    private double exposure = 1100;
    private long exposureRange = 100;
    private float gain = 255;
    private long gainRange = 100;
    private long lastMs = -1;
    private boolean isStreaming = false;

    @Override
    public void loop() {
        boolean wasStreaming = isStreaming;
        isStreaming = portal.getCameraState() == VisionPortal.CameraState.STREAMING;

        long time = System.currentTimeMillis();
        if (lastMs == -1) {
            lastMs = time;
        }
        long delta = time - lastMs;
        lastMs = time;

        if (isStreaming) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            TimeUnit timeUnit = TimeUnit.MICROSECONDS;
            if (!wasStreaming) {
                // exposure = exposureControl.getExposure(timeUnit);
                exposureRange = exposureControl.getMaxExposure(timeUnit) - exposureControl.getMinExposure(timeUnit);
            }
            exposureControl.setExposure(Math.round(exposure), timeUnit);

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (!wasStreaming) {
                // gain = gainControl.getGain();
                gainRange = gainControl.getMaxGain() - gainControl.getMinGain();
            }
            gainControl.setGain(Math.round(gain));
        }

        exposure += Math.pow(gamepad1.left_stick_x, 3) * delta / 1000 * exposureRange / 50;
        gain += (float) (Math.pow(gamepad1.right_stick_x, 3) * delta / 1000 * gainRange / 5);
        telemetry.addData("papajp", gainRange);
        telemetry.addData("exposure", exposure);
        telemetry.addData("gain", gain);

        List<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 21) {
                telemetry.addData("goon", detection.center);
            }
        }

        inputManager.update();
        dash.update();
    }
}