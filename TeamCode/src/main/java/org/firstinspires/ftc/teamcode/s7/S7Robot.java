package org.firstinspires.ftc.teamcode.s7;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class S7Robot {
    public enum PoseEstimateMode {
        KEEP,
        RESET
    }

    public S7Drive s7Drive;
    public PoseEstimateMode poseEstimateMode;

    private LinearOpMode opMode;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private TfodProcessor tfod;
    private static final String DEFAULT_PROP_MODEL = "PropModel1.tflite";
    private static final String[] LABELS = {"PropBlue", "PropRed"};

    public S7Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.s7Drive = new S7Drive(opMode.hardwareMap);
        this.poseEstimateMode = PoseEstimateMode.KEEP;
    }

    public S7Robot(LinearOpMode opMode, Pose2d startPose) {
        this.opMode = opMode;
        this.s7Drive = new S7Drive(opMode.hardwareMap, startPose);
        this.poseEstimateMode = PoseEstimateMode.KEEP;
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

    public ArrayList<AprilTagDetection> getAprilTagDetections() {
        return aprilTagProcessor.getDetections();
    }

    //Tensorflow
    public void initTensorFlow(float minConfidence) {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(DEFAULT_PROP_MODEL)
                .setModelLabels(LABELS)
                .build();
        tfod.setMinResultConfidence(minConfidence);

        VisionPortal visionPortal = new VisionPortal.Builder()
            .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(1280, 720))
            .addProcessor(tfod)
            .build();
    }

    public void initTensorFlow(float minConfidence, String model) {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(model)
                .setModelLabels(LABELS)
                .build();
        tfod.setMinResultConfidence(minConfidence);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(tfod)
                .build();
    }

    public List<Recognition> getTensorFlowRecognitions() {
        return tfod.getRecognitions();
    }

    public Recognition findRecognitionWithLabel(String label) {
        for (Recognition recognition : tfod.getRecognitions()) {
            if (recognition.getLabel() == label) {
                return recognition;
            }
        }

        return null;
    }
}
