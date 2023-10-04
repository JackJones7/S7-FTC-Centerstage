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

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


/*
* UNTITLED TRAJECTORY SEQUENCE NOTATION 1
*
* lineTo: lt x y
* lineToConstantHeading: ltch x y
* lineToLinearHeading: ltlh x y endHeading
* lineToSplineHeading: ltsh x y endHeading
* strafeTo: stt x y
* forward: f distance
* back: b distance
* strafeLeft: sl distance
* strafeRight: sr distance
* splineTo: st x y splineHeading
* splineToConstantHeading: stch x y splineHeading
* splineToLinearHeading: stlh x y endHeading splineHeading
* splineToSplineHeading: stsh x y endHeading splineHeading
* turn: t radians
*
* All angles are in radians. You'll have to convert them yourself. Sorry.
*
*/


@ExportClassToBlocks
public class TrajectorySequenceParser {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);


    @ExportToBlocks(
            tooltip = "Converts string into trajectory sequence using the notation I made up",
            parameterLabels = {"Start Pose", "Sequence"}
    )
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
                    sequence.lineToSplineHeading(new Pose2d(Double.parseDouble(params[0]), Double.parseDouble(params[1]), Double.parseDouble(params[2])));
                    break;

                case "stt":
                    sequence.strafeTo(new Vector2d(Double.parseDouble(params[0]), Double.parseDouble(params[1])));
                    break;

                case "f":
                    sequence.forward(Double.parseDouble(params[0]));
                    break;

                case "b":
                    sequence.back(Double.parseDouble(params[0]));
                    break;

                case "sl":
                    sequence.strafeLeft(Double.parseDouble(params[0]));
                    break;

                case "sr":
                    sequence.strafeRight(Double.parseDouble(params[0]));
                    break;

                case "st":
                    sequence.splineTo(new Vector2d(Double.parseDouble(params[0]), Double.parseDouble(params[1])), Double.parseDouble(params[2]));
                    break;

                case "stch":
                    sequence.splineToConstantHeading(new Vector2d(Double.parseDouble(params[0]), Double.parseDouble(params[1])), Double.parseDouble(params[2]));
                    break;

                case "stlh":
                    sequence.splineToLinearHeading(new Pose2d(Double.parseDouble(params[0]), Double.parseDouble(params[1]), Double.parseDouble(params[2])), Double.parseDouble(params[3]));
                    break;

                case "stsh":
                    sequence.splineToSplineHeading(new Pose2d(Double.parseDouble(params[0]), Double.parseDouble(params[1]), Double.parseDouble(params[2])), Double.parseDouble(params[3]));
                    break;

                case "t":
                    sequence.turn(Double.parseDouble(params[0]));
                    break;
            }

        }
        return sequence.build();
    }

}
