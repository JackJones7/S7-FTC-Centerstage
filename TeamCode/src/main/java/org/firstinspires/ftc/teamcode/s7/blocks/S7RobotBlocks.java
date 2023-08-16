package org.firstinspires.ftc.teamcode.s7.blocks;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.s7.S7Robot;

@ExportClassToBlocks
public class S7RobotBlocks extends BlocksOpModeCompanion {

    //TODO: setWeightedDrivePower

    //TODO: lineTo

    //TODO: lineToConstantHeading

    //TODO: lineToLinearHeading

    //TODO: lineToSplineHeading

    //TODO: strafeTo

    @ExportToBlocks(
            tooltip = "S7Robot constructor"
    )
    public static S7Robot s7Robot() {
        return new S7Robot(linearOpMode);
    }

    @ExportToBlocks(
            tooltip = "Move the robot forward",
            parameterLabels = {"S7Robot", "Distance"}
    )
    public static void forward(S7Robot robot, double distance) {
        robot.forward(distance);
    }

    @ExportToBlocks(
            tooltip = "Move the robot forward",
            parameterLabels = {"S7Robot", "Distance"}
    )
    public static void back(S7Robot robot, double distance) {
        robot.back(distance);
    }

    @ExportToBlocks(
            tooltip = "Move the robot forward",
            parameterLabels = {"S7Robot", "Distance"}
    )
    public static void strafeLeft(S7Robot robot, double distance) {
        robot.strafeLeft(distance);
    }

    @ExportToBlocks(
            tooltip = "Move the robot forward",
            parameterLabels = {"S7Robot", "Distance"}
    )
    public static void strafeRight(S7Robot robot, double distance) {
        robot.strafeRight(distance);
    }

    //TODO: splineTo

    //TODO: splineToConstantHeading

    //TODO: splineToLinearHeading

    //TODO: splineToSplineHeading

    //TODO: turn

    //TODO: Some system for following trajectories and trajectory sequences
}
