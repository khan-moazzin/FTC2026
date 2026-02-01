package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

/**
 * Encoder-dominant localizer with Limelight AprilTag correction.
 */
public class FusedLocalizer implements Localizer {

    private final DriveEncoderLocalizer odom;
    private final LimelightVision limelight;

    private Pose pose = new Pose(0, 0, 0);

    // Tune later
    private static final double VISION_WEIGHT = 0.1;

    public FusedLocalizer(
            DriveEncoderLocalizer odom,
            LimelightVision limelight
    ) {
        this.odom = odom;
        this.limelight = limelight;
    }

    @Override
    public void update() {
        // Encoder update
        odom.update();
        pose = odom.getPose();

        // Vision correction
        if (limelight.hasPose()) {
            LimelightVision.Pose2d v = limelight.getFieldPose();

            Pose corrected = new Pose(
                    lerp(pose.getX(), v.x, VISION_WEIGHT),
                    lerp(pose.getY(), v.y, VISION_WEIGHT),
                    angleLerp(pose.getHeading(), v.heading, VISION_WEIGHT)
            );
            pose = corrected;
            odom.setPose(corrected);
        }}

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return odom.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odom.setStartPose(setStart);
        pose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odom.setPose(setPose);
        pose = setPose;
    }

    @Override
    public double getTotalHeading() {
        return odom.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odom.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odom.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odom.getTurningMultiplier();
    }

    @Override
    public void resetIMU() {}

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading());
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double angleLerp(double a, double b, double t) {
        double diff = Math.atan2(Math.sin(b - a), Math.cos(b - a));
        return a + diff * t;
    }
}
