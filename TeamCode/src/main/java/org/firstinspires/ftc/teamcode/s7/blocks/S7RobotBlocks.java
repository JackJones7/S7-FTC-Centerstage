package org.firstinspires.ftc.teamcode.s7.blocks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.s7.S7Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@ExportClassToBlocks
public class S7RobotBlocks extends BlocksOpModeCompanion {

    //constructors
    @ExportToBlocks(
            tooltip = "S7Robot constructor"
    )
    public static S7Robot s7Robot() {
        return new S7Robot(linearOpMode);
    }

    @ExportToBlocks(
            tooltip = "S7Robot constructor with start pose",
            parameterLabels = {"Start Pose"}
    )
    public static S7Robot s7Robot(Pose2d startPose) {return new S7Robot(linearOpMode, startPose);}


    //drive
    @ExportToBlocks(
            heading = "Set Weighted Drive Power",
            tooltip = "Set drive power based on a Pose2d (See RoadRunnerBlocks)",
            parameterLabels = {"S7Robot", "Drive Power"}
    )
    public static void setWeightedDrivePower(S7Robot robot, Pose2d drivePower) {
        robot.s7Drive.setWeightedDrivePower(drivePower);
    }

    @ExportToBlocks(
            heading = "Drive Control",
            tooltip = "Basic drive controls based on SetWeightedDrivePower",
            parameterLabels = {"S7Robot", "Power"}
    )
    public static void driveControl(S7Robot robot, double power) {
        robot.s7Drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
    }

    @ExportToBlocks(
            heading = "Load trajectory",
            tooltip = "Load a trajectory from a .yaml file. Set Red Flipped to true to flip for red side",
            parameterLabels = {"S7Robot", "Filename", "Red Flipped"}
    )
    public static Trajectory loadTrajectory(S7Robot robot, String filename, boolean redFlip) {
        return robot.s7Drive.loadTrajectory(filename, redFlip);
    }

    @ExportToBlocks(
            heading = "Set Maximum Velocity",
            tooltip = "Set maximum velocity for generated/loaded trajectories",
            parameterLabels = {"S7Robot", "Max Velocity"}
    )
    public static void setMaxVel(S7Robot robot, double maxVel) {
        robot.s7Drive.setMaxVel(maxVel);
    }

    @ExportToBlocks(
            heading = "Set Maximum Angular Velocity",
            tooltip = "Set maximum angular velocity for generated/loaded trajectories",
            parameterLabels = {"S7Robot", "Max Ang Velocity"}
    )
    public static void setMaxAngVel(S7Robot robot, double maxAngVel) {
        robot.s7Drive.setMaxAngVel(maxAngVel);
    }

    @ExportToBlocks(
            heading = "Set Maximum Acceleration",
            tooltip = "Set maximum acceleration for generated/loaded trajectories",
            parameterLabels = {"S7Robot", "Max acceleration"}
    )
    public static void setMaxAccel(S7Robot robot, double maxAccel) {
        robot.s7Drive.setMaxAccel(maxAccel);
    }

    @ExportToBlocks(
            heading = "Set Maximum Angular Acceleration",
            tooltip = "Set maximum angular acceleration for generated/loaded trajectories",
            parameterLabels = {"S7Robot", "Max angular acceleration"}
    )
    public static void setMaxAngAccel(S7Robot robot, double maxAngAccel) {
        robot.s7Drive.setMaxAngAccel(maxAngAccel);
    }


    @ExportToBlocks(
            tooltip = "Straight line with a tangent heading",
            parameterLabels = {"S7Robot", "Position", "Reversed"}
    )
    public static Trajectory lineTo(S7Robot robot, Vector2d position, boolean reversed) {
        return robot.s7Drive.lineTo(position, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line with a constant heading",
            parameterLabels = {"S7Robot", "Position", "Reversed"}
    )
    public static Trajectory lineToConstantHeading(S7Robot robot, Vector2d position, boolean reversed) {
        return robot.s7Drive.lineToConstantHeading(position, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line with linear heading interpolation",
            parameterLabels = {"S7Robot", "Position", "Reversed"}
    )
    public static Trajectory lineToLinearHeading(S7Robot robot, Pose2d position, boolean reversed) {
        return robot.s7Drive.lineToLinearHeading(position, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line with spline heading interpolation",
            parameterLabels = {"S7Robot", "Position", "Reversed"}
    )
    public static Trajectory lineToSplineHeading(S7Robot robot, Pose2d position, boolean reversed) {
        return robot.s7Drive.lineToSplineHeading(position, reversed);
    }

    @ExportToBlocks(
            tooltip = "Strafe with tangent heading",
            parameterLabels = {"S7Robot", "Position", "Reversed"}
    )
    public static Trajectory strafeTo(S7Robot robot, Vector2d position, boolean reversed) {
        return robot.s7Drive.strafeTo(position, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line towards current heading",
            parameterLabels = {"S7Robot", "Distance", "Reversed"}
    )
    public static Trajectory forward(S7Robot robot, double distance, boolean reversed) {
        return robot.s7Drive.forward(distance, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line away from current heading",
            parameterLabels = {"S7Robot", "Distance", "Reversed"}
    )
    public static Trajectory back(S7Robot robot, double distance, boolean reversed) {
        return robot.s7Drive.back(distance, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line left of current heading",
            parameterLabels = {"S7Robot", "Distance", "Reversed"}
    )
    public static Trajectory strafeLeft(S7Robot robot, double distance, boolean reversed) {
        return robot.s7Drive.strafeLeft(distance, reversed);
    }

    @ExportToBlocks(
            tooltip = "Straight line right of current heading",
            parameterLabels = {"S7Robot", "Distance", "Reversed"}
    )
    public static Trajectory strafeRight(S7Robot robot, double distance, boolean reversed) {
        return robot.s7Drive.strafeRight(distance, reversed);
    }

    @ExportToBlocks(
            tooltip = "Spline with tangent heading",
            parameterLabels = {"S7Robot", "Position", "End Tangent", "Reversed"}
    )
    public static Trajectory splineTo(S7Robot robot, Vector2d position, double endTangent, boolean reversed) {
        return robot.s7Drive.splineTo(position, endTangent, reversed);
    }

    @ExportToBlocks(
            tooltip = "Spline with constant heading",
            parameterLabels = {"S7Robot", "Position", "End Tangent", "Reversed"}
    )
    public static Trajectory splineToConstantHeading(S7Robot robot, Vector2d position, double endTangent, boolean reversed) {
        return robot.s7Drive.splineToConstantHeading(position, endTangent, reversed);
    }

    @ExportToBlocks(
            tooltip = "Spline with linear heading interpolation",
            parameterLabels = {"S7Robot", "Position", "End Tangent", "Reversed"}
    )
    public static Trajectory splineToLinearHeading(S7Robot robot, Pose2d position, double endTangent, boolean reversed) {
        return robot.s7Drive.splineToLinearHeading(position, endTangent, reversed);
    }

    @ExportToBlocks(
            tooltip = "Spline with spline heading interpolation",
            parameterLabels = {"S7Robot", "Position", "End Tangent", "Reversed"}
    )
    public static Trajectory splineToSplineHeading(S7Robot robot, Pose2d position, double endTangent, boolean reversed) {
        return robot.s7Drive.splineToSplineHeading(position, endTangent, reversed);
    }

    @ExportToBlocks(
            tooltip = "Turn the robot by a specified angle (in radians)",
            comment = "Angle is measured in radians. Use block in RoadRunnerBlocks to convert to degrees",
            parameterLabels = {"S7Robot", "Angle"}
    )
    public static void turn(S7Robot robot, double angle) {
        robot.s7Drive.drive.turn(angle);
    }

    @ExportToBlocks(
            tooltip = "Follow a trajectory sequence",
            parameterLabels = {"S7Robot", "Trajectory sequence"}
    )
    public static void followTrajectorySequence(S7Robot robot, TrajectorySequence sequence) {
        robot.s7Drive.followTrajectorySequence(sequence);
    }

    @ExportToBlocks(
            tooltip = "Follow a trajectory",
            parameterLabels = {"S7Robot", "Trajectory"}
    )
    public static void followTrajectory(S7Robot robot, Trajectory trajectory) {
        robot.s7Drive.followTrajectory(trajectory);
    }


    @ExportToBlocks(
            heading = "Init arm assembly",
            tooltip = "Initialize internal arm/claw controls",
            parameterLabels = {"S7Robot"}
    )
    public static void initArmAssembly(S7Robot robot) {
        robot.initArmAssembly();
    }

    @ExportToBlocks(
            heading = "Set arm constraints",
            tooltip = "Set min and max angle for arm in degrees (Use Init arm assembly first)",
            parameterLabels = {"S7Robot", "Min degrees", "Max degrees"}
    )
    public static void setArmConstraints(S7Robot robot, double min, double max) {
        robot.s7ArmAssembly.setArmConstraints(min, max);
    }

    @ExportToBlocks(
            heading = "Set arm angle",
            tooltip = "Set arm angle in degrees (Use Init arm assembly first)",
            parameterLabels = {"S7Robot", "Angle"}
    )
    public static void setArmAngle(S7Robot robot, double angle) {
        robot.s7ArmAssembly.setArmAngle(angle);
    }

    @ExportToBlocks(
            heading = "Set wrist constraints",
            tooltip = "Set min and max angle for wrist in degrees (Use Init arm assembly first)",
            parameterLabels = {"S7Robot", "Min degrees", "Max degrees"}
    )
    public static void setWristConstraints(S7Robot robot, double min, double max) {
        robot.s7ArmAssembly.setWristConstraints(min, max);
    }

    @ExportToBlocks(
            heading = "Set relative wrist angle",
            tooltip = "Set wrist angle in degrees, relative to arm angle (Use Init arm assembly first)",
            parameterLabels = {"S7Robot", "Angle"}
    )
    public static void setWristAngleRelative(S7Robot robot, double angle) {
        robot.s7ArmAssembly.setWristAngleRelative(angle);
    }


    @ExportToBlocks(
            tooltip = "Initialize internal AprilTag processing",
            parameterLabels = {"S7Robot"}
    )
    public static void initAprilTag(S7Robot robot) {
        robot.initAprilTag();
    }

    @ExportToBlocks(
            tooltip = "Get current april tag detections (Must use initAprilTag first)",
            parameterLabels = {"S7Robot"}
    )
    public static ArrayList<AprilTagDetection> getAprilTagDetections(S7Robot robot) {
        return robot.getAprilTagDetections();
    }

    @ExportToBlocks(
            tooltip = "Initialize TensorFlow object detection",
            parameterLabels = {"S7Robot", "Minimum Confidence"}
    )
    public static void initTensorFlow(S7Robot robot, float minConfidence){
        robot.initTensorFlow(minConfidence);
    }

    @ExportToBlocks(
            tooltip = "Initialize TensorFlow object detection",
            parameterLabels = {"S7Robot", "Minimum Confidence", "Model Asset Name"}
    )
    public static void initTensorFlow(S7Robot robot, float minConfidence, String model){
        robot.initTensorFlow(minConfidence, model);
    }

    @ExportToBlocks(
            tooltip = "Get list of objects recognized by TensorFlow",
            parameterLabels = {"S7Robot"}
    )
    public static List<Recognition> getTensorFlowRecognitions(S7Robot robot) {
        return robot.getTensorFlowRecognitions();
    }

    @ExportToBlocks(
            tooltip = "Find a specific recognized object based on its label",
            parameterLabels = {"S7Robot", "Label"}
    )
    public static Recognition findRecognitionWithLabel(S7Robot robot, String label){
        return robot.findRecognitionWithLabel(label);
    }
}
