package org.firstinspires.ftc.teamcode.s7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import java.io.File;

public class S7Drive {

    public SampleMecanumDrive drive;


    private TrajectoryVelocityConstraint VEL_CONSTRAINT;
    private TrajectoryAccelerationConstraint ACCEL_CONSTRAINT;


    public S7Drive(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.VEL_CONSTRAINT = drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
        this.ACCEL_CONSTRAINT = drive.getAccelerationConstraint(MAX_ACCEL);
        this.drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    public S7Drive(HardwareMap hardwareMap, double maxVel, double maxAngVel, double maxAccel) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.VEL_CONSTRAINT = drive.getVelocityConstraint(maxVel, maxAngVel, TRACK_WIDTH);
        this.ACCEL_CONSTRAINT = drive.getAccelerationConstraint(maxAccel);
        this.drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    public S7Drive(HardwareMap hardwareMap, Pose2d startPose) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.VEL_CONSTRAINT = drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
        this.ACCEL_CONSTRAINT = drive.getAccelerationConstraint(MAX_ACCEL);
        this.drive.setPoseEstimate(startPose);
    }

    public S7Drive(HardwareMap hardwareMap, double maxVel, double maxAngVel, double maxAccel, Pose2d startPose) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.VEL_CONSTRAINT = drive.getVelocityConstraint(maxVel, maxAngVel, TRACK_WIDTH);
        this.ACCEL_CONSTRAINT = drive.getAccelerationConstraint(maxAccel);
        this.drive.setPoseEstimate(startPose);
    }

    //Drive controls
    public void setWeightedDrivePower(Pose2d power) {
        drive.setWeightedDrivePower(power);
    }


    //Follow trajectories
    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void followTrajectorySequence(TrajectorySequence sequence) {
        drive.followTrajectorySequence(sequence);
    }

    //Load/create trajectories
    private TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public void setVelConstraint(double maxVel, double maxAngVel) {
        VEL_CONSTRAINT = drive.getVelocityConstraint(maxVel, maxAngVel, TRACK_WIDTH);
    }

    public void setAccelConstraint(double maxAccel) {
        ACCEL_CONSTRAINT = drive.getAccelerationConstraint(maxAccel);
    }

    public Trajectory loadTrajectory(String filename) {
        File file = AppUtil.getInstance().getSettingsFile(filename)
        return TrajectoryConfigManager.load(file);
    }

    public Trajectory lineTo(Vector2d endPosition, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).lineTo(endPosition).build();
    }

    public Trajectory lineToConstantHeading(Vector2d endPosition, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).lineToConstantHeading(endPosition).build();
    }

    public Trajectory lineToLinearHeading(Pose2d endPose, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).lineToLinearHeading(endPose).build();
    }

    public Trajectory lineToSplineHeading(Pose2d endPose, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).lineToSplineHeading(endPose).build();
    }

    public Trajectory strafeTo(Vector2d endPosition, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).strafeTo(endPosition).build();
    }

    public Trajectory splineTo(Vector2d endPosition, double endTangent, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).splineTo(endPosition, endTangent).build();
    }

    public Trajectory splineToConstantHeading(Vector2d endPosition, double endTangent, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).splineToConstantHeading(endPosition, endTangent).build();
    }

    public Trajectory splineToLinearHeading(Pose2d endPose, double endTangent, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).splineToLinearHeading(endPose, endTangent).build();
    }

    public Trajectory splineToSplineHeading(Pose2d endPose, double endTangent, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).splineToSplineHeading(endPose, endTangent).build();
    }

    public Trajectory forward(double distance, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).forward(distance).build();
    }

    public Trajectory back(double distance, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).back(distance).build();
    }

    public Trajectory strafeLeft(double distance, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).strafeLeft(distance).build();
    }

    public Trajectory strafeRight(double distance, boolean reversed) {
        return trajectoryBuilder(drive.getPoseEstimate(), reversed).strafeRight(distance).build();
    }
}
