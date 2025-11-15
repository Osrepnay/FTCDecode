package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static final double INTAKE_OFF = 0;
    public static final double INTAKE_ON = 0.6;
    public static final double INTAKE_TRANSFER = 0.22;
    public static final double INTAKE_HOLD = 0.5;
    public static final double INTAKE_BARELYMOVE = 0.1;

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
        if (pow == INTAKE_HOLD) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        intake.setPower(pow);
    }
}
