package org.firstinspires.ftc.teamcode.s7;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class S7Robot {
    public SampleMecanumDrive drive;

    private LinearOpMode opMode;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public S7Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.drive = new SampleMecanumDrive(opMode.hardwareMap);
    }

    public S7Robot(LinearOpMode opMode, Pose2d startPose) {
        this.opMode = opMode;
        this.drive = new SampleMecanumDrive(opMode.hardwareMap);
        this.drive.setPoseEstimate(startPose);
    }

    //drive functions
    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    public void lineTo(Vector2d endPosition) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(endPosition)
                .build());
    }

    public void lineToConstantHeading(Vector2d endPosition) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(endPosition)
                .build());
    }

    public void lineToLinearHeading(Pose2d endPose) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(endPose)
                .build());
    }

    public void lineToSplineHeading(Pose2d endPose) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(endPose)
                .build());
    }

    public void strafeTo(Vector2d endPosition) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(endPosition)
                .build());
    }

    public void forward(double distance) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(distance)
                .build());
    }

    public void back(double distance) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(distance)
                .build());
    }

    public void strafeLeft(double distance) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(distance)
                .build());
    }

    public void strafeRight(double distance) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(distance)
                .build());
    }

    public void splineTo(Vector2d endPosition, double endTangent) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(endPosition, endTangent)
                .build());
    }

    public void splineToConstantHeading(Vector2d endPosition, double endTangent) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(endPosition, endTangent)
                .build());
    }

    public void splineToLinearHeading(Pose2d endPose, double endTangent) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(endPose, endTangent)
                .build());
    }

    public void splineToSplineHeading(Pose2d endPose, double endTangent) {
        followTrajectoryAndWait(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(endPose, endTangent)
                .build());
    }

    public void turn(double angle) {
        drive.turn(angle);
        waitForDrive();
    }

    //TODO: add overrides for different max velocity and accel

    public void followTrajectorySequence(TrajectorySequence sequence) {
        drive.followTrajectorySequence(sequence);
        waitForDrive();
    }

    public void waitForDrive() {
        drive.waitForIdle();
    }

    public void followTrajectoryAndWait(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
        waitForDrive();
    }

    //AprilTag functionality
    public void initAprilTag() {
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessor = aprilTagProcessorBuilder.build();

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.addProcessor(aprilTagProcessor);
        visionPortal = visionPortalBuilder.build();
    }

}
