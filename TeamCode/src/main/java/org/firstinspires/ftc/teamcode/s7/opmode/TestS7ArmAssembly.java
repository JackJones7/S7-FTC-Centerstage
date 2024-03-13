package org.firstinspires.ftc.teamcode.s7.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.s7.S7Robot;

@Config
@Autonomous (name = "TEST-S7 Arm Assembly")
public class TestS7ArmAssembly extends LinearOpMode {

    public static double WRIST_ANGLE = 145;
    public static double WRIST_MIN_ANGLE = 145;
    public static double WRIST_MAX_ANGLE = 300;

    public static double ARM_ANGLE = 140;
    public static double ARM_MIN_ANGLE = 140;
    public static double ARM_MAX_ANGLE = 180;

    private S7Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new S7Robot(this);
        robot.initArmAssembly();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.s7ArmAssembly.setWristConstraints(WRIST_MIN_ANGLE, WRIST_MAX_ANGLE);
            robot.s7ArmAssembly.setWristAngleRelative(WRIST_ANGLE);

            robot.s7ArmAssembly.setArmConstraints(ARM_MIN_ANGLE, ARM_MAX_ANGLE);
            robot.s7ArmAssembly.setArmAngle(ARM_ANGLE);
        }
    }

}
