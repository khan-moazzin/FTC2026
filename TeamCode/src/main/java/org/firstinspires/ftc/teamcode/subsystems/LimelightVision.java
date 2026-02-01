package org.firstinspires.ftc.teamcode.subsystems;

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

    public Pose2d getFieldPose() {
        Pose3D p = limelight.getLatestResult().getBotpose();
        return new Pose2d(
                p.getPosition().x * METERS_TO_INCHES,
                p.getPosition().y * METERS_TO_INCHES,
                Math.toRadians(p.getOrientation().getYaw())
        );
    }

    public static class Pose2d {
        public final double x, y, heading;
        public Pose2d(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.heading = h;
        }
    }
}
