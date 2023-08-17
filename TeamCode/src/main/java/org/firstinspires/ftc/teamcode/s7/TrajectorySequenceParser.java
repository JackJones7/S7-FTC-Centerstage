package org.firstinspires.ftc.teamcode.s7;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
*
* This is a class for an idea I had. If this class were finished, it would take a string that represented a code for a
* trajectory sequence and convert it into a usable trajectory sequence. It was supposed to make its usage in blocks easier.
* While I'm not working on it anymore, it could still be useful so I'm keeping the class around for now
*
*/

public class TrajectorySequenceParser {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);


    public static TrajectorySequence parseSequence(Pose2d startPose, String sequenceText) {
        TrajectorySequenceBuilder sequence = new TrajectorySequenceBuilder(startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL);

        String[] sequenceList = sequenceText.split("\n");

        //TODO: Add error handling
        for (String line : sequenceList) {
            String[] current = line.split(" ");

            switch(current[0]) {
                case "lt":
                    //sequence.lineTo(new Vector2d(current[1], current[2]));
            }
        }
        return null;
    }

}
