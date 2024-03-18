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
            tooltip = "Returns a VisionPortal with a Tensorflow processor, configured for our autonomous",
            heading = "Init VisionPortal with TensorFlow"
    )
    public static VisionPortal initVisionWithTf() {
        String[] labels = {"PropBlue", "PropRed"};

        TfodProcessor tfod = new TfodProcessor.Builder()
                .setModelAssetName("PropModel1.tflite")
                .setModelLabels(labels)
                .build();

        VisionPortal vision = new VisionPortal.Builder()
                .setCamera(BlocksOpModeCompanion.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1080, 720))
                .addProcessor(tfod)
                .build();

        return vision;
    }

    public static Recognition lookForLabel(TfodProcessor tfod, String targetLabel) {
        Recognition result;
        ElapsedTime lookTimer = new ElapsedTime();
        lookTimer.reset();

        while (lookTimer.time(TimeUnit.SECONDS) <= 3) {
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
