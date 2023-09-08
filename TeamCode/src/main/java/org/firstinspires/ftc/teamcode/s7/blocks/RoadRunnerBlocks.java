package org.firstinspires.ftc.teamcode.s7.blocks;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@ExportClassToBlocks
public class RoadRunnerBlocks extends BlocksOpModeCompanion {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);

    //Data types
    @ExportToBlocks(
            tooltip = "Pose2d constructor",
            parameterLabels = {"X", "Y", "Heading"}
    )
    public static Pose2d pose2d(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    @ExportToBlocks(
            tooltip = "Vector2d constructor",
            parameterLabels = {"X", "Y"}
    )
    public static Vector2d vector2d(double x, double y) {
        return new Vector2d(x, y);
    }

    //Math
    @ExportToBlocks(
            tooltip = "Convert degrees to radians",
            parameterLabels = {"Degrees"}
    )
    public static double toRadians(double degrees) {return Math.toRadians(degrees);}

    @ExportToBlocks(
            tooltip = "Convert radians to degrees",
            parameterLabels = {"Radians"}
    )
    public static double toDegrees(double radians) {return Math.toDegrees(radians);}

    //Trajectory Sequence builder
    @ExportToBlocks(
            tooltip = "TrajectorySequenceBuilder constructor",
            parameterLabels = {"Start Pose"}
    )
    public static TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    @ExportToBlocks(
            tooltip = "Line to a position in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "End Position"}
    )
    public static void lineTo(TrajectorySequenceBuilder builder, Vector2d endPosition) {
        builder.lineTo(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Line to a position with constant heading in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "End Position"}
    )
    public static void lineToConstantHeading(TrajectorySequenceBuilder builder, Vector2d endPosition) {
        builder.lineToConstantHeading(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Line to a position and heading with linear heading in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "End Pose"}
    )
    public static void lineToLinearHeading(TrajectorySequenceBuilder builder, Pose2d endPose) {
        builder.lineToLinearHeading(endPose);
    }

    @ExportToBlocks(
            tooltip = "Line to a position and heading with spline heading in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "End Pose"}
    )
    public static void lineToSplineHeading(TrajectorySequenceBuilder builder, Pose2d endPose) {
        builder.lineToSplineHeading(endPose);
    }

    @ExportToBlocks(
            tooltip = "Strafe to a position in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "End Pose"}
    )
    public static void strafeTo(TrajectorySequenceBuilder builder, Vector2d endPosition) {
        builder.strafeTo(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Move forward in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "Distance"}
    )
    public static void forward(TrajectorySequenceBuilder builder, double distance) {
        builder.forward(distance);
    }

    @ExportToBlocks(
            tooltip = "Move backward in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "Distance"}
    )
    public static void back(TrajectorySequenceBuilder builder, double distance) {
        builder.back(distance);
    }

    @ExportToBlocks(
            tooltip = "Strafe left in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "Distance"}
    )
    public static void strafeLeft(TrajectorySequenceBuilder builder, double distance) {
        builder.strafeLeft(distance);
    }

    @ExportToBlocks(
            tooltip = "Strafe left in a trajectory sequence",
            parameterLabels = {"Trajectory Sequence Builder", "Distance"}
    )
    public static void strafeRight(TrajectorySequenceBuilder builder, double distance) {
        builder.strafeRight(distance);
    }

    @ExportToBlocks(
            tooltip = "Spline to position in a trajectory sequence",
            parameterLabels = {"Trajectory sequence builder", "End Position", "End Tangent"}
    )
    public static void splineTo(TrajectorySequenceBuilder builder, Vector2d endPosition, double endTangent) {
        builder.splineTo(endPosition, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Spline to position with constant heading in a trajectory sequence",
            parameterLabels = {"Trajectory sequence builder", "End Position", "End Tangent"}
    )
    public static void splineToConstantHeading(TrajectorySequenceBuilder builder, Vector2d endPosition, double endTangent) {
        builder.splineToConstantHeading(endPosition, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Spline to position with linear heading in a trajectory sequence",
            parameterLabels = {"Trajectory sequence builder", "End Pose", "End Tangent"}
    )
    public static void splineToLinearHeading(TrajectorySequenceBuilder builder, Pose2d endPose, double endTangent) {
        builder.splineToLinearHeading(endPose, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Spline to position with spline heading in a trajectory sequence",
            parameterLabels = {"Trajectory sequence builder", "End Pose", "End Tangent"}
    )
    public static void splineToSplineHeading(TrajectorySequenceBuilder builder, Pose2d endPose, double endTangent) {
        builder.splineToSplineHeading(endPose, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Turn in a trajectory sequence (Angle in radians)",
            parameterLabels = {"Trajectory Sequence Builder", "Angle"}
    )
    public static void turn(TrajectorySequenceBuilder builder, double angle) {
        builder.turn(angle);
    }

    @ExportToBlocks(
            tooltip = "Build trajectory sequence",
            parameterLabels = {"Trajectory sequence builder"}
    )
    public static TrajectorySequence build(TrajectorySequenceBuilder builder) {
        return builder.build();
    }

}
