package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * PoseEstimator - fuses encoder velocities and camera measurements using a Kalman filter.
 * IMU is handled externally.
 */
public class PoseEstimator {

    private final DriveEncoderLocalizer odom;
    private final Camera camera;

    // 5-state Kalman filter: [x, y, theta, vx, vy]
    private final double[] x = new double[5];
    private final double[][] P = new double[5][5];
    private final double[][] Q = new double[5][5]; // process noise
    private final double[][] R_vision = new double[3][3]; // vision measurement
    private boolean initialized = false;

    public PoseEstimator(DriveEncoderLocalizer odom, Camera camera) {
        this.odom = odom;
        this.camera = camera;
        initCovariances();
    }

    private void initCovariances() {
        for (int i = 0; i < 5; i++) {
            P[i][i] = 1000;
            Q[i][i] = (i < 3) ? 0.2 : 1.0; // process noise: position smaller, velocity larger
        }
        R_vision[0][0] = 0.5;
        R_vision[1][1] = 0.5;
        R_vision[2][2] = 0.2; // vision heading uncertainty
    }

    /**
     * Update the estimator using odometry velocities and camera detections.
     * IMU is not used here.
     *
     * @param dt elapsed time in seconds since last update
     */
    public void update(double dt) {
        Pose odomPose = odom.getPose();
        Pose odomVel = odom.getVelocity();

        if (!initialized) {
            x[0] = odomPose.getX();
            x[1] = odomPose.getY();
            x[2] = odomPose.getHeading();
            x[3] = odomVel.getX();
            x[4] = odomVel.getY();
            initialized = true;
            return;
        }

        predict(dt);

        // correct using odometry velocities
        double[][] Hv = new double[2][5];
        Hv[0][3] = 1; Hv[1][4] = 1;
        double[][] Rv = {{0.5,0},{0,0.5}};
        correct(Hv, Rv, new double[]{odomVel.getX(), odomVel.getY()});

        // correct using camera detections if available
        List<AprilTagDetection> dets = camera.getDetections();
        if (!dets.isEmpty()) {
            double avgX = 0, avgY = 0, avgTheta = 0;
            for (AprilTagDetection d : dets) {
                avgX += d.robotPose.getPosition().x;
                avgY += d.robotPose.getPosition().y;
                avgTheta += Math.toRadians(d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            }
            int n = dets.size();
            avgX /= n; avgY /= n; avgTheta /= n;

            double[][] Hcam = {
                    {1, 0, 0, 0, 0},
                    {0, 1, 0, 0, 0},
                    {0, 0, 1, 0, 0}
            };
            correct(Hcam, R_vision, new double[]{avgX, avgY, avgTheta});
        }
    }

    /** Predict step for constant velocity model */
    private void predict(double dt) {
        x[0] += x[3] * dt;
        x[1] += x[4] * dt;
        x[2] = wrapAngle(x[2]);

        // P = P + Q for constant velocity
        for (int i = 0; i < 5; i++) P[i][i] += Q[i][i];
    }

    /**
     * Linear Kalman filter correction
     *
     * @param H measurement matrix
     * @param R measurement noise
     * @param z measurement vector
     */
    private void correct(double[][] H, double[][] R, double[] z) {
        int m = z.length;
        int n = x.length;

        // Compute innovation y = z - Hx
        double[] y = new double[m];
        for (int i = 0; i < m; i++) {
            y[i] = z[i];
            for (int j = 0; j < n; j++) y[i] -= H[i][j] * x[j];
        }

        // Compute S = H * P * H^T + R
        double[][] S = new double[m][m];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                for (int k = 0; k < n; k++) S[i][j] += H[i][k] * P[k][j]; // H*P
                double sum = 0;
                for (int k = 0; k < n; k++) sum += H[i][k] * P[k][k] * H[j][k];
                S[i][j] += R[i][j]; // add R
            }
        }

        // Compute Kalman gain K = P * H^T * S^-1
        double[][] K = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                double sum = 0;
                for (int k = 0; k < m; k++) {
                    if (S[k][k] != 0) sum += P[i][j] / S[k][k];
                }
                K[i][j] = sum;
            }
        }

        // Update state x = x + K*y
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) x[i] += K[i][j] * y[j];
        }

        // Update covariance P = (I - K*H)*P
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0;
                for (int k = 0; k < m; k++) sum += K[i][k] * H[k][j];
                P[i][j] = (I[i][j] - sum) * P[i][j];
            }
        }

        // wrap angle
        x[2] = wrapAngle(x[2]);
    }

    public Pose getPose() {
        return new Pose(x[0], x[1], x[2], odom.getPose().getCoordinateSystem());
    }

    private static double wrapAngle(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
}
