package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Catapult;
import org.firstinspires.ftc.teamcode.subsystems.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

@TeleOp(name = "TeleopMain", group = "Teleop")
public class TeleopMain extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private Robot mRobot;
    private LimelightVision limelight;

    // ===== AUTO ALIGN CONSTANTS =====
    private static final double TARGET_DISTANCE_IN = 12.0;
    private static final double STRAFE_kP  = 0.035;
    private static final double FORWARD_kP = 0.045;
    private static final double ROTATE_kP  = 0.025;

    @Override
    public void init() {
        mRobot = new Robot();
        mRobot.init(hardwareMap, telemetry);
        mRobot.drive.startTeleOpDrive(true);

        limelight = new LimelightVision(hardwareMap);

        while (mRobot.catapult.state == Catapult.State.RETURNING) {
            mRobot.catapult.update();
        }
        mRobot.catapult.hold();
        mRobot.toggleRobotOriented();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    private boolean lastCata = false;

    @Override
    public void loop() {

        // ================= DRIVER INPUTS =================
        boolean intakeIn = gamepad1.right_trigger > 0.2;
        boolean intakeOut = gamepad1.right_bumper || gamepad2.left_bumper;
        boolean footOut = gamepad2.a;
        boolean footUp = gamepad2.b;
        boolean cataUp = gamepad2.right_bumper;
        boolean cataDown = gamepad2.right_trigger > 0.2;
        boolean cataReleased = !cataDown && lastCata;

        if (intakeIn && intakeOut) intakeIn = false;
        if (footOut && footUp) footOut = false;
        lastCata = cataDown;
        if (cataUp && cataDown) cataUp = false;

        if (intakeIn) mRobot.intake.intake();
        else if (intakeOut) mRobot.intake.outtake();
        else mRobot.intake.stop();

        if (footOut) mRobot.foot.lower();
        else if (footUp) mRobot.foot.raise();
        else mRobot.foot.stop();

        if (cataUp) mRobot.catapult.fire();
        else if (cataDown) mRobot.catapult.load();
        else if (cataReleased) mRobot.catapult.hold();

        if (gamepad1.start) mRobot.resetHeading();


        // ================= AUTO ALIGN =================
        boolean autoAlign = gamepad1.left_trigger > 0.2;

        if (autoAlign && limelight.hasTarget()) {

            double tx = limelight.getTx();
            double distance = limelight.getForwardDistanceInches();

            if (Math.abs(tx) < 0.3) tx = 0;
            if (distance < 1.0) distance = 0;

            double strafe = -tx * STRAFE_kP;

            double forward = distance * FORWARD_kP;

            double rotate = -tx * ROTATE_kP;

            mRobot.drive(
                    clamp(forward, 0.0, 0.55),
                    clamp(strafe, -0.2, 0.2),
                    clamp(rotate, -0.1, 0.1)
            );
        }
 else  {
            mRobot.drive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.6
            );
        }




        mRobot.update();
        telemetry.addData("AutoAlign", autoAlign && limelight.hasTarget());
        telemetry.addData("tx", limelight.getTx());
        telemetry.addData("ty", limelight.getTy());
        telemetry.addData("distance", limelight.getForwardDistanceInches());
        telemetry.update();

    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
