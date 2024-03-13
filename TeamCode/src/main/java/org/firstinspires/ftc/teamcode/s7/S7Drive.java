package org.firstinspires.ftc.teamcode.s7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig.Waypoint;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class S7Drive {

    public SampleMecanumDrive drive;


    private double MAX_VEL = DriveConstants.MAX_VEL;
    private double MAX_ANG_VEL = DriveConstants.MAX_ANG_VEL;
    private double MAX_ACCEL = DriveConstants.MAX_ACCEL;
    private double MAX_ANG_ACCEL = DriveConstants.MAX_ANG_ACCEL;
    private TrajectoryGroupConfig groupConfig;

    private TrajectoryVelocityConstraint VEL_CONSTRAINT;
    private TrajectoryAccelerationConstraint ACCEL_CONSTRAINT;


    public S7Drive(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        updateConstraints();
        this.drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    public S7Drive(HardwareMap hardwareMap, double maxVel, double maxAngVel, double maxAccel, double maxAngAccel) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.MAX_VEL = maxVel; this.MAX_ANG_VEL = maxAngVel; this.MAX_ACCEL = maxAccel; this.MAX_ANG_ACCEL = maxAngAccel;
        updateConstraints();
        this.drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    public S7Drive(HardwareMap hardwareMap, Pose2d startPose) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        updateConstraints();
        this.drive.setPoseEstimate(startPose);
    }

    public S7Drive(HardwareMap hardwareMap, double maxVel, double maxAngVel, double maxAccel, double maxAngAccel, Pose2d startPose) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.MAX_VEL = maxVel; this.MAX_ANG_VEL = maxAngVel; this.MAX_ACCEL = maxAccel; this.MAX_ANG_ACCEL = maxAngAccel;
        updateConstraints();
        this.drive.setPoseEstimate(startPose);
    }

    private void updateConstraints() {
        VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);
        groupConfig = new TrajectoryGroupConfig(
                MAX_VEL,
                MAX_ACCEL,
                MAX_ANG_VEL,
                MAX_ANG_ACCEL,
                14.0,
                15.0,
                TrajectoryGroupConfig.DriveType.GENERIC,
                DriveConstants.TRACK_WIDTH,
                0.0,
                1.0
        );
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

    public void setMaxVel(double maxVel) {
        MAX_VEL = maxVel;
        updateConstraints();
    }

    public void setMaxAngVel(double maxAngVel) {
        MAX_ANG_VEL = maxAngVel;
        updateConstraints();
    }

    public void setMaxAccel(double maxAccel) {
        MAX_ACCEL = maxAccel;
        updateConstraints();
    }

    public void setMaxAngAccel(double maxAngAccel) {
        MAX_ANG_ACCEL = maxAngAccel;
        updateConstraints();
    }

    public TrajectoryConfig redFlip(TrajectoryConfig config) {
        Pose2d startPose = new Pose2d(config.getStartPose().getX() * -1,
                config.getStartPose().getY(),
                config.getStartPose().getHeading() + (Math.toRadians(90) - config.getStartPose().getHeading()) * 2);
        double startTangent = config.getStartTangent() + Math.PI;

        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (Waypoint waypoint : config.getWaypoints()) {
            Vector2d position = new Vector2d(waypoint.getPosition().getX() * -1, waypoint.getPosition().getY());
            double heading = waypoint.getHeading() + (Math.toRadians(90) - waypoint.getHeading()) * 2;
            double tangent = (waypoint.getInterpolationType() == TrajectoryConfig.HeadingInterpolationType.TANGENT) ? waypoint.getTangent() : waypoint.getTangent() + Math.PI;
            waypoints.add(new Waypoint(position, heading, tangent, waypoint.getInterpolationType()));
        }

        return new TrajectoryConfig(startPose, startTangent, config.getWaypoints(), config.getResolution());
    }

    public Trajectory loadTrajectory(String filename) {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        return TrajectoryConfigManager.loadConfig(file).toTrajectory(groupConfig);
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
