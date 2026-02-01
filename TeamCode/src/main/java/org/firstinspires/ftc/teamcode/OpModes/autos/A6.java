package org.firstinspires.ftc.teamcode.OpModes.autos;

import static org.firstinspires.ftc.teamcode.Constants.FIRE_TIME;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Catapult;

@Autonomous(name = "A6", group = "Autonomous")
@Configurable
public class A6 extends OpMode {

    private static final double INTAKE_TIME = 3;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private A6Path paths;
    private final ElapsedTime StateTimer = new ElapsedTime();
    private final Robot mRobot = new Robot();

    // Track last state to start paths only once
    private int lastState = -1;

    // Timer for advanceAfter
    private final ElapsedTime advanceTimer = new ElapsedTime();
    private boolean isWaiting = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        mRobot.init(hardwareMap, telemetry);
        follower = mRobot.drive;

        follower.setStartingPose(new Pose(21.513, 122.293, Math.toRadians(143.5)));
        paths = new A6Path(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        while (mRobot.catapult.state == Catapult.State.RETURNING) {
            mRobot.catapult.update();
        }
    }

    @Override
    public void start() {
        advanceTimer.reset();
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
        mRobot.update();

        try {
            follower.updateErrorAndVectors();
            if (follower.getCurrentPath() != null) {
                telemetry.addData("Velocity", follower.poseTracker.getVelocity().getMagnitude() <
                        follower.getCurrentPath().getPathEndVelocityConstraint());
                telemetry.addData("Translation", follower.poseTracker.getPose().distanceFrom(
                        follower.getClosestPose().getPose()) <
                        follower.getCurrentPath().getPathEndTranslationalConstraint());
                telemetry.addData("Heading", MathFunctions.getSmallestAngleDifference(
                        follower.poseTracker.getPose().getHeading(),
                        follower.getClosestPointHeadingGoal()) <
                        follower.getCurrentPath().getPathEndHeadingConstraint());
            }
            telemetry.addData("Busy", follower.isBusy());
        } catch (Exception ignored) {
            telemetry.addData("Velocity", "ERROR");
            telemetry.addData("Translation", "ERROR");
            telemetry.addData("Heading", "ERROR");
            telemetry.addData("Busy", "ERROR");
        }

        panelsTelemetry.debug("PathState", pathState);
        panelsTelemetry.update(telemetry);
    }
    // -----------------------------------------------------
    // AUTONOMOUS STATE MACHINE
    // -----------------------------------------------------
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Assume preloaded
                mRobot.catapult.fire();
                advanceAfter(FIRE_TIME);
                break;

            case 1:
                follow(paths.Path1);
                advanceAfterPath();
                break;

            case 2:
                mRobot.intake.intake();
                follow(paths.Path2);
                advanceAfterPath();
                break;

            case 3:
                follow(paths.Path3);
                if(follower.isBusy())
                    break;
                advanceAfter(3);
                break;

            case 4:
                follow(paths.Path4);
                advanceAfterPath();
                break;
            case 5:
                mRobot.intake.stop();
                mRobot.catapult.fire();
                advanceAfter(FIRE_TIME);
                break;
            case 6:

                break;
        }
    }

    // -----------------------------------------------------
    // PATH FOLLOWING
    // -----------------------------------------------------
    private void follow(PathChain path, boolean holdPoint){
        if (pathState != lastState) {
            follower.followPath(path, holdPoint);
            lastState = pathState;
        }
    }

    private void follow(PathChain path){
        follow(path, false);
    }

    // -----------------------------------------------------
    // ADVANCE HELPERS
    // -----------------------------------------------------
    private void advance() {
        pathState++;
        StateTimer.reset();
        isWaiting = false;
    }

    private void advanceWhen(boolean cond) {
        if (cond) advance();
    }

    private void advanceAfterPath() {
        advanceWhen(!follower.isBusy());
    }

    private void advanceAfterPathWithMin(double minimumTime) {
        if (StateTimer.seconds() > minimumTime)
            advanceAfterPath();
    }

    private void advanceAfter(double seconds) {
        if (!isWaiting) advanceTimer.reset();
        isWaiting = true;
        advanceWhen(advanceTimer.seconds() > seconds);
        if (advanceTimer.seconds() > seconds)
            isWaiting = false;
    }
}
