package org.firstinspires.ftc.teamcode.s7.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.s7.S7Robot;

@Config
@Autonomous (name = "TEST-S7 Arm Assembly")
public class TestS7ArmAssembly extends LinearOpMode {

    public static double ANGLE = 150;

    private S7Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new S7Robot(this);
        robot.initArmAssembly();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.s7ArmAssembly.setWristAngleAbsolute(ANGLE);
        }
    }

}
