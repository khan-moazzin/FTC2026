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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Catapult;

@Autonomous(name = "A6 RED", group = "Autonomous")
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

        // Initialize robot normally
        mRobot.init(hardwareMap, telemetry);

        // Use the Constants.createFollower so it gets the FusedLocalizer
        follower = Constants.createFollower(hardwareMap, telemetry);

        // Set starting pose
        Pose startPose = new Pose(21.513, 122.293, Math.toRadians(143.5)).mirror();
        follower.setStartingPose(startPose);

        paths = new A6Path(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        // Make sure catapult finishes returning before start
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
        follower.update();
        autonomousPathUpdate();
        mRobot.update();

        Pose fusedPose = follower.poseTracker.getPose();

        // Telemetry
        telemetry.addData("X", fusedPose.getX());
        telemetry.addData("Y", fusedPose.getY());
        telemetry.addData("Heading", Math.toDegrees(fusedPose.getHeading()));

        if (follower.getCurrentPath() != null) {
            telemetry.addData("Velocity", follower.poseTracker.getVelocity().getMagnitude() <
                    follower.getCurrentPath().getPathEndVelocityConstraint());
            telemetry.addData("Translation", fusedPose.distanceFrom(
                    follower.getClosestPose().getPose()) <
                    follower.getCurrentPath().getPathEndTranslationalConstraint());
            telemetry.addData("Heading", MathFunctions.getSmallestAngleDifference(
                    fusedPose.getHeading(),
                    follower.getClosestPointHeadingGoal()) <
                    follower.getCurrentPath().getPathEndHeadingConstraint());
            telemetry.addData("Busy", follower.isBusy());
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
                follow(paths.Path1,true);
                advanceAfterPath();
                break;

            case 2:
                mRobot.intake.intake();
                follow(paths.Path2);
                advanceAfterPathWithMin(3.5);
                break;

            case 3:
                follow(paths.Path3);
                advanceAfterPathWithMin(3.5);
                break;

            case 4:
                mRobot.catapult.fire();
                advanceAfter(FIRE_TIME);
                break;
            case 5:
                follow(paths.leave,true);
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
