package org.firstinspires.ftc.teamcode.s7;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class S7ArmAssembly {
    private Servo armPivotServoLeft;
    private Servo armPivotServoRight;
    private Servo wristPivotServo;
    private Servo clawServoLeft;
    private Servo clawServoRight;

    private final double ARM_PIVOT_WEIGHT = 1.0 / 300.0;
    private final double WRIST_PIVOT_WEIGHT = 1.0 / 300.0;

    private double armPivotMinDegrees = 0;
    private double armPivotMaxDegrees = 300;
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

        wristPivotServo.setDirection(Servo.Direction.REVERSE);
        armPivotServoRight.setDirection(Servo.Direction.REVERSE);
        clawServoRight.setDirection(Servo.Direction.REVERSE);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    public void setArmConstraints(double minDegrees, double maxDegrees) {
        armPivotMinDegrees = minDegrees;
        armPivotMaxDegrees = maxDegrees;
    }

    public void setArmAngle(double degrees) {
        armPivotServoLeft.setPosition(clamp(degrees, armPivotMinDegrees, armPivotMaxDegrees) * ARM_PIVOT_WEIGHT);
        armPivotServoRight.setPosition(clamp(degrees, armPivotMinDegrees, armPivotMaxDegrees) * ARM_PIVOT_WEIGHT);
    }

    public void setWristConstraints(double minDegrees, double maxDegrees) {
        wristPivotMinDegrees = minDegrees;
        wristPivotMaxDegrees = maxDegrees;
    }

    public void setWristAngleRelative(double degrees) {
        wristAngleType = AngleType.ABSOLUTE;
        wristPivotServo.setPosition(clamp(degrees, wristPivotMinDegrees, wristPivotMaxDegrees) * WRIST_PIVOT_WEIGHT);
    }

    //TODO: Set absolute wrist angle (also while you're here don't forget to update the RC app)

}
