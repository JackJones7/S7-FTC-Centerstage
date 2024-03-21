package org.firstinspires.ftc.teamcode.s7.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.s7.S7Robot;

@Config
@TeleOp(name="S7Robot TeleOp test")
@Disabled
public class TestS7RobotTeleOp extends LinearOpMode {
    public static double DISTANCE = 48;

    private S7Robot robot;

    @Override
    public void runOpMode() {
        robot = new S7Robot(this, new Pose2d(-61.01, -61.39, Math.toRadians(90.00)));

        waitForStart();

        //while(opModeIsActive()) {
        //    robot.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
        //}
    }
}
