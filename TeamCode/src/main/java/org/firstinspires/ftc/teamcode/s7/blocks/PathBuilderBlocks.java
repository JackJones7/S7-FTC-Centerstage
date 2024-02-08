package org.firstinspires.ftc.teamcode.s7.blocks;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@ExportClassToBlocks
public class PathBuilderBlocks {
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);

    @ExportToBlocks(
            tooltip = "Returns a PathBuilder",
            parameterLabels = {"Start Pose"}
    )
    public static PathBuilder pathBuilder(Pose2d startPose) {
        return new PathBuilder(startPose);
    }

    @ExportToBlocks(
            tooltip = "Add a lineTo segment to a path builder",
            parameterLabels = {"Path builder", "End position"}
    )
    public static void lineTo(PathBuilder builder, Vector2d endPosition) {
        builder.lineTo(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Add a lineToConstantHeading segment to a path builder",
            parameterLabels = {"Path builder", "End position"}
    )
    public static void lineToConstantHeading(PathBuilder builder, Vector2d endPosition) {
        builder.lineToConstantHeading(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Add a lineToLinearHeading segment to a path builder",
            parameterLabels = {"Path builder", "End pose"}
    )
    public static void lineToLinearHeading(PathBuilder builder, Pose2d endPose) {
        builder.lineToLinearHeading(endPose);
    }

    @ExportToBlocks(
            tooltip = "Add a lineToSplineHeading segment to a path builder",
            parameterLabels = {"Path builder", "End pose"}
    )
    public static void lineToSplineHeading(PathBuilder builder, Pose2d endPose) {
        builder.lineToSplineHeading(endPose);
    }

    @ExportToBlocks(
            tooltip = "Add a strafeTo segment to a path builder",
            parameterLabels = {"Path builder", "End position"}
    )
    public static void strafeTo(PathBuilder builder, Vector2d endPosition) {
        builder.strafeTo(endPosition);
    }

    @ExportToBlocks(
            tooltip = "Add a forward segment to a path builder",
            parameterLabels = {"Path builder", "Distance"}
    )
    public static void forward(PathBuilder builder, double distance) {
        builder.forward(distance);
    }

    @ExportToBlocks(
            tooltip = "Add a back segment to a path builder",
            parameterLabels = {"Path builder", "Distance"}
    )
    public static void back(PathBuilder builder, double distance) {
        builder.back(distance);
    }

    @ExportToBlocks(
            tooltip = "Add a strafeLeft segment to a path builder",
            parameterLabels = {"Path builder", "Distance"}
    )
    public static void strafeLeft(PathBuilder builder, double distance) {
        builder.strafeLeft(distance);
    }

    @ExportToBlocks(
            tooltip = "Add a strafeRight segment to a path builder",
            parameterLabels = {"Path builder", "Distance"}
    )
    public static void strafeRight(PathBuilder builder, double distance) {
        builder.strafeRight(distance);
    }

    @ExportToBlocks(
            tooltip = "Add a splineTo segment to a path builder",
            parameterLabels = {"Path builder", "End position", "End Tangent"}
    )
    public static void splineTo(PathBuilder builder, Vector2d endPosition, double endTangent) {
        builder.splineTo(endPosition, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Add a splineToConstantHeading segment to a path builder",
            parameterLabels = {"Path builder", "End position", "End Tangent"}
    )
    public static void splineToConstantHeading(PathBuilder builder, Vector2d endPosition, double endTangent) {
        builder.splineToConstantHeading(endPosition, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Add a splineToLinearHeading segment to a path builder",
            parameterLabels = {"Path builder", "End pose", "End Tangent"}
    )
    public static void splineToLinearHeading(PathBuilder builder, Pose2d endPose, double endTangent) {
        builder.splineToLinearHeading(endPose, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Add a splineToSplineHeading segment to a path builder",
            parameterLabels = {"Path builder", "End pose", "End Tangent"}
    )
    public static void splineToSplineHeading(PathBuilder builder, Pose2d endPose, double endTangent) {
        builder.splineToSplineHeading(endPose, endTangent);
    }

    @ExportToBlocks(
            tooltip = "Build a path builder",
            parameterLabels = {"Path builder"}
    )
    public static Path build(PathBuilder builder) {
        return builder.build();
    }

    @ExportToBlocks(
            tooltip = "Create a trajectory from a path",
            parameterLabels = {"Path"}
    )
    public static Trajectory buildTrajectoryFromPath(Path path) {
        return TrajectoryGenerator.INSTANCE.generateTrajectory(path, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    @ExportToBlocks(
            tooltip = "Create a trajectory from a path builder",
            parameterLabels = {"Path builder"}
    )
    public static Trajectory buildTrajectoryFromPathBuilder(PathBuilder pathBuilder) {
        Path path = pathBuilder.build();
        return TrajectoryGenerator.INSTANCE.generateTrajectory(path, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

}
