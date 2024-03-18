package org.firstinspires.ftc.teamcode.s7.blocks;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@ExportClassToBlocks
public class TensorFlowBlocks {

    @ExportToBlocks(
            tooltip = "Returns a TfodProcessor configured for autonomous",
            heading = "Init TensorFlow"
    )
    public static TfodProcessor initTfod() {
        String[] labels = {"PropBlue", "PropRed"};

        TfodProcessor tfod = new TfodProcessor.Builder()
                .setModelAssetName("PropModel1.tflite")
                .setModelLabels(labels)
                .build();

        return tfod;
    }

    @ExportToBlocks(
            tooltip = "Returns a VisionPortal with given TfodProcessor, configured for autonomous",
            heading = "Init VisionPortal",
            parameterLabels = {"Tensorflow processor"}
    )
    public static VisionPortal initVisionWithTfod(TfodProcessor tfod) {
        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(BlocksOpModeCompanion.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(tfod)
                .build();

        return vision;
    }

    @ExportToBlocks(
            tooltip = "Halt OpMode until object with given label is found or maximum time is elapsed",
            heading = "Stop and look for label",
            parameterLabels = {"Tensorflow Processor", "Target label", "Max seconds"}
    )
    public static Recognition lookForLabel(TfodProcessor tfod, String targetLabel, double maxSeconds) {
        Recognition result;
        ElapsedTime lookTimer = new ElapsedTime();
        lookTimer.reset();

        while (lookTimer.time(TimeUnit.SECONDS) <= maxSeconds) {
            for (Recognition recognition : tfod.getRecognitions()) {
                if (recognition.getLabel() == targetLabel) {
                    return recognition;
                }
            }

            BlocksOpModeCompanion.linearOpMode.idle();
        }

        return null;
    }

}
