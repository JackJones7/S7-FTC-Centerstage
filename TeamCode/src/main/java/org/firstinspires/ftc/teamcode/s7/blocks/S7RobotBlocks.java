package org.firstinspires.ftc.teamcode.s7.blocks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.s7.S7Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@ExportClassToBlocks
public class S7RobotBlocks extends BlocksOpModeCompanion {

    @ExportToBlocks(
            tooltip = "S7Robot constructor"
    )
    public static S7Robot s7Robot() {
        return new S7Robot(linearOpMode);
    }

    //TODO: add headings

    //drive
    @ExportToBlocks(
            heading = "Set Weighted Drive Power",
            tooltip = "Set drive power based on a Pose2d (See RoadRunnerBlocks)",
            parameterLabels = {"S7Robot", "Drive Power"}
    )
    public static void setWeightedDrivePower(S7Robot robot, Pose2d drivePower) {
        robot.setWeightedDrivePower(drivePower);
    }

    @ExportToBlocks(
            tooltip = "Move robot to a point in a straight line with a tangent heading",
            parameterLabels = {"S7Robot", "Position"}
    )
    public static void lineTo(S7Robot robot, Vector2d position) {
        robot.lineTo(position);
    }

    @ExportToBlocks(
            tooltip = "Move robot to a point in a straight line with a constant heading",
            parameterLabels = {"S7Robot", "Position"}
    )
    public static void lineToConstantHeading(S7Robot robot, Vector2d position) {
        robot.lineToConstantHeading(position);
    }

    @ExportToBlocks(
            tooltip = "Move robot to a point in a straight line with a linear heading",
            parameterLabels = {"S7Robot", "Position"}
    )
    public static void lineToLinearHeading(S7Robot robot, Pose2d position) {
        robot.lineToLinearHeading(position);
    }

    @ExportToBlocks(
            tooltip = "Move robot to a point in a straight line with a spline heading",
            parameterLabels = {"S7Robot", "Position"}
    )
    public static void lineToSplineHeading(S7Robot robot, Pose2d position) {
        robot.lineToSplineHeading(position);
    }

    @ExportToBlocks(
            tooltip = "Strafe robot to a point",
            parameterLabels = {"S7Robot", "Position"}
    )
    public static void strafeTo(S7Robot robot, Vector2d position) {
        robot.strafeTo(position);
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

    @ExportToBlocks(
            tooltip = "Move robot in a spline to an end position",
            parameterLabels = {"S7Robot", "Position", "End Tangent"}
    )
    public static void splineTo(S7Robot robot, Vector2d position, double endTangent) {
        robot.splineTo(position, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Move robot in a spline to an end position with a constant heading",
            parameterLabels = {"S7Robot", "Position", "End Tangent"}
    )
    public static void splineToConstantHeading(S7Robot robot, Vector2d position, double endTangent) {
        robot.splineToConstantHeading(position, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Move robot in a spline to an end position with a linear heading",
            parameterLabels = {"S7Robot", "Position", "End Tangent"}
    )
    public static void splineToLinearHeading(S7Robot robot, Pose2d position, double endTangent) {
        robot.splineToLinearHeading(position, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Move robot in a spline to an end position with a spline heading",
            parameterLabels = {"S7Robot", "Position", "End Tangent"}
    )
    public static void splineToSplineHeading(S7Robot robot, Pose2d position, double endTangent) {
        robot.splineToSplineHeading(position, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Turn the robot by a specified angle (in radians)",
            comment = "Angle is measured in radians. Use block in RoadRunnerBlocks to convert to degrees",
            parameterLabels = {"S7Robot", "Angle"}
    )
    public static void turn(S7Robot robot, double angle) {
        robot.turn(angle);
    }

    @ExportToBlocks(
            tooltip = "Follow a trajectory sequence",
            parameterLabels = {"S7Robot", "Trajectory sequence"}
    )
    public static void followTrajectorySequence(S7Robot robot, TrajectorySequence sequence) {
        robot.followTrajectorySequence(sequence);
    }
}
