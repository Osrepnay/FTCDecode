package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static final double INTAKE_OFF = 0;
    public static final double INTAKE_ON = 0.3;
    public static final double INTAKE_TRANSFER = 0.2;

    public final DcMotorEx intake;

    public Intake(DcMotorEx intake) {
        this.intake = intake;
        // theoretically makes transferring more reliable
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, "intake"));
    }

    public void setPower(double pow) {
        intake.setPower(pow);
    }
}
