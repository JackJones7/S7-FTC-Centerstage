package org.firstinspires.ftc.teamcode.s7;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class S7ArmAssembly {
    private Servo armPivotServoLeft;
    private Servo armPivotServoRight;
    private Servo wristPivotServo;
    private Servo clawServoLeft;
    private Servo clawServoRight;

    private final double ARM_PIVOT_WEIGHT = 1 / 300;
    private final double WRIST_PIVOT_WEIGHT = 1 / 300;
    private double currentArmPivotWeight = ARM_PIVOT_WEIGHT;
    private double currentWristPivotWeight = WRIST_PIVOT_WEIGHT;

    private double wristPivotMinDegrees = 0;
    private double wristPivotMaxDegrees = 300;

    private AngleType wristAngleType = AngleType.ABSOLUTE;

    public enum AngleType {
        ABSOLUTE,
        RELATIVE
    }

    public S7ArmAssembly(HardwareMap hardwareMap) {
        armPivotServoLeft = hardwareMap.get(Servo.class, "LeftUpperPivot");
        armPivotServoRight = hardwareMap.get(Servo.class, "RightUpperPivot");
        wristPivotServo = hardwareMap.get(Servo.class, "LowerPivot");
        clawServoLeft = hardwareMap.get(Servo.class, "LeftClaw");
        clawServoRight = hardwareMap.get(Servo.class, "RightClaw");
    }


    public void setWristConstraints(double minDegrees, double maxDegrees) {
        wristPivotServo.scaleRange(minDegrees * WRIST_PIVOT_WEIGHT, maxDegrees * WRIST_PIVOT_WEIGHT);
        currentWristPivotWeight = 1 / (maxDegrees - minDegrees);
        wristPivotMinDegrees = minDegrees;
        wristPivotMaxDegrees = maxDegrees;
    }

    public void setWristAngleAbsolute(double degrees) {
        wristAngleType = AngleType.ABSOLUTE;
        wristPivotServo.setPosition(degrees * currentWristPivotWeight);
    }

}
