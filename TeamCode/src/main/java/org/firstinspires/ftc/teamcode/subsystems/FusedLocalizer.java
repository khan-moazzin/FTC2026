package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Custom PedroLocalizer using encoder and camera fusion via PoseEstimator.
 */
public class FusedLocalizer implements Localizer {

    private final DriveEncoderLocalizer odom;
    private final PoseEstimator estimator;
    private long lastUpdateTime = System.nanoTime();

    public FusedLocalizer(HardwareMap hardwareMap, DriveEncoderLocalizer odom, Camera camera) {
        this.odom = odom;
        camera.init(hardwareMap);
        this.estimator = new PoseEstimator(odom, camera);
    }

    /**
     * Call periodically in loop or opmode.
     * Reads IMU internally for heading fusion.
     */
    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastUpdateTime) / 1e9; // seconds
        lastUpdateTime = now;
        estimator.update(dt);
    }

    @Override
    public Pose getPose() {
        return estimator.getPose();
    }

    @Override
    public Pose getVelocity() {
        Pose odomVel = odom.getVelocity();
        return new Pose(odomVel.getX(), odomVel.getY(),odomVel.getHeading()); // ignore angular velocity
    }

    @Override
    public Vector getVelocityVector() {
        Pose vel = getVelocity();
        return vel.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odom.setStartPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        odom.setPose(setPose);
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
    public void resetIMU(){}

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    @Override
    public boolean isNAN() {
        Pose p = getPose();
        return Double.isNaN(p.getX()) || Double.isNaN(p.getY()) || Double.isNaN(p.getHeading());
    }
}
