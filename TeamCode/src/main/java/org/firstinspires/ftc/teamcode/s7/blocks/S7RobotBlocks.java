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

    @ExportToBlocks(
            tooltip = "S7Robot constructor with pose estimate mode",
            parameterLabels = {"Pose Estimate Mode"}
    )
    public static S7Robot s7Robot(S7Robot.PoseEstimateMode poseEstimateMode) {return new S7Robot(linearOpMode, poseEstimateMode);}

    @ExportToBlocks(
            tooltip = "S7Robot constructor with start pose and pose estimate mode",
            parameterLabels = {"Start Pose", "Pose Estimate Mode"}
    )
    public static S7Robot s7Robot(Pose2d startPose, S7Robot.PoseEstimateMode poseEstimateMode) {return new S7Robot(linearOpMode, startPose, poseEstimateMode);}


    //Pose Estimate Mode
    @ExportToBlocks(
            tooltip = "Keep pose estimate after movement"
    )
    public static S7Robot.PoseEstimateMode keepMode() {return S7Robot.PoseEstimateMode.KEEP;}

    @ExportToBlocks(
            tooltip = "Reset pose estimate after movement"
    )
    public static S7Robot.PoseEstimateMode resetMode() {return S7Robot.PoseEstimateMode.RESET;}

    //TODO: Add headings
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
            heading = "Drive Control",
            tooltip = "Basic drive controls based on SetWeightedDrivePower",
            parameterLabels = {"S7Robot", "Power"}
    )
    public static void driveControl(S7Robot robot, double power) {
        robot.driveControl(gamepad1, power);
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

    @ExportToBlocks(
            tooltip = "Follow a trajectory",
            parameterLabels = {"S7Robot", "Trajectory"}
    )
    public static void followTrajectory(S7Robot robot, Trajectory trajectory) {
        robot.followTrajectory(trajectory);
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
