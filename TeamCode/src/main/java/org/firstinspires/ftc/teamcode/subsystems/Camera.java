package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Camera subsystem - handles only AprilTag processing.
 */
public class Camera {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean enabled = false;

    public void init(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH,
                        AngleUnit.DEGREES)
                .setCameraPose(new Position(
                        DistanceUnit.INCH,0.0,0.0,0.0, 0
                ), new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0)) // adjust as needed
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        enabled = true;
    }

    public void setEnabled(boolean enable) {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, enable);
            enabled = enable;
        }
    }

    public List<AprilTagDetection> getDetections() {
        if (!enabled || aprilTag == null) return new ArrayList<>();
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return detections == null ? new ArrayList<>() : detections;
    }

}
