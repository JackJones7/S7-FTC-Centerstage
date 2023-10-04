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
            String[] currentLine = line.split(" ");
            String code = currentLine[0];
            String[] params = java.util.Arrays.copyOfRange(currentLine, 1, currentLine.length);

            switch (code) {
                case "lt":
                    sequence.lineTo(new Vector2d(Double.parseDouble(params[0]), Double.parseDouble(params[1])));
                    break;

                case "ltch":
                    sequence.lineToConstantHeading(new Vector2d(Double.parseDouble(params[0]), Double.parseDouble(params[1])));
                    break;

                case "ltlh":
                    sequence.lineToLinearHeading(new Pose2d(Double.parseDouble(params[0]), Double.parseDouble(params[1]), Double.parseDouble(params[2])));
                    break;

                case "ltsh":
                    break;

                case "stt":
                    break;

                case "f":
                    break;

                case "b":
                    break;

                case "sl":
                    break;

                case "sr":
                    break;

                case "st":
                    break;

                case "stch":
                    break;

                case "stlh":
                    break;

                case "stsh":
                    break;

                case "t":
                    break;
            }

        }
        return sequence.build();
    }

}
