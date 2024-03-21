package org.firstinspires.ftc.teamcode.s7.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.s7.S7Robot;

@Config
@TeleOp(name="S7Robot Auto test")
@Disabled
public class TestS7RobotAuto extends LinearOpMode {
    public static double DISTANCE = 48;

    private S7Robot robot;

    @Override
    public void runOpMode() {
        robot = new S7Robot(this, new Pose2d(-61.01, -61.39, Math.toRadians(90.00)));

        waitForStart();

        /*
        Export from RRPathGen

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-61.01, -61.39, Math.toRadians(90.00)))
        .lineToConstantHeading(new Vector2d(-35.81, -35.81))
        .lineToConstantHeading(new Vector2d(-59.51, -12.58))
        .splineTo(new Vector2d(-47.21, 6.53), Math.toRadians(0.00))
        .splineTo(new Vector2d(-34.91, -5.77), Math.toRadians(-90.00))
        .build();
        */
    }
}
