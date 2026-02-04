package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightVision {

    private static final double METERS_TO_INCHES = 39.3701;
    private final Limelight3A limelight;

    public LimelightVision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public boolean hasPose() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid() && r.getBotpose() != null;
    }

    public Pose getFieldPose() {
        Pose3D p = limelight.getLatestResult().getBotpose();
        Pose pose =  new Pose(
                p.getPosition().x * METERS_TO_INCHES,
                p.getPosition().y * METERS_TO_INCHES,
                Math.toRadians(p.getOrientation().getYaw()));

                return FTCCoordinates.INSTANCE.convertToPedro(pose);
    }

    public static class Pose2d {
        public final double x, y, heading;
        public Pose2d(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.heading = h;
        }


    }

    // ================= AUTO ALIGN =================

    private static final double CAMERA_HEIGHT_IN = 10.0;   // camera height from floor
    private static final double TAG_HEIGHT_IN    = 5.5;    // AprilTag center height
    private static final double CAMERA_PITCH_DEG = 15.0;   // camera tilt downwards

    public boolean hasTarget() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    public double getTx() {
        return limelight.getLatestResult().getTx(); // degrees
    }

    public double getTy() {
        return limelight.getLatestResult().getTy(); // degrees
    }

    public double getForwardDistanceInches() {
        double ty = limelight.getLatestResult().getTy();
        double totalAngleDeg = CAMERA_PITCH_DEG + ty;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        double distance =
                (CAMERA_HEIGHT_IN - TAG_HEIGHT_IN) / Math.tan(totalAngleRad);

        // Invert so: closer → smaller ty → smaller distance feels WRONG
        // We want: closer → smaller distance
        return Math.abs(distance);
    }

    public double getYawRadians() {
        return Math.toRadians(limelight.getLatestResult().getTx());
    }




}

