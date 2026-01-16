package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Color;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp
public class IMURecalibrate extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        requestOpModeStop();
    }
}
