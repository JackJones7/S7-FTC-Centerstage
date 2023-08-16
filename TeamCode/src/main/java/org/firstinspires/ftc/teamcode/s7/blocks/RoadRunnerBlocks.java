package org.firstinspires.ftc.teamcode.s7.blocks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

@ExportClassToBlocks
public class RoadRunnerBlocks extends BlocksOpModeCompanion {

    @ExportToBlocks(
            tooltip = "Pose2d constructor"
    )
    public static Pose2d pose2d(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    @ExportToBlocks(
            tooltip = "Vector2d constructor"
    )
    public static Vector2d vector2d(double x, double y) {
        return new Vector2d(x, y);
    }

}
