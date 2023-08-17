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
            tooltip = "Build trajectory sequence",
            parameterLabels = {"Trajectory sequence builder"}
    )
    public static TrajectorySequence build(TrajectorySequenceBuilder builder) {
        return builder.build();
    }

    //TODO: Add rest

}
