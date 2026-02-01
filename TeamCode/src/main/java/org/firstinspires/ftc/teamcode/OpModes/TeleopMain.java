package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.subsystems.Catapult;
import org.firstinspires.ftc.teamcode.subsystems.Foot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleopMain", group = "Teleop")
public class TeleopMain extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    Robot mRobot = new Robot();

    @Override
    public void init() {

        // Initialize robot
        mRobot.init(hardwareMap, telemetry);
        mRobot.drive.startTeleOpDrive(true);


        // Ensure catapult finishes returning
        while (mRobot.catapult.state == Catapult.State.RETURNING) {
            mRobot.catapult.update();
            mRobot.sendTelemetry();
            telemetry.update();
        }
        mRobot.catapult.hold();
        mRobot.sendTelemetry();
        telemetry.update();
        mRobot.toggleRobotOriented();
    }


    @Override
    public void start() {
        runtime.reset();
    }
    boolean lastCata = false;
    @Override
    public void loop() {

        boolean intakeInButton = gamepad1.right_trigger > 0.2;
        boolean intakeOutButton = gamepad2.left_bumper || gamepad1.right_bumper;
        if (intakeInButton && intakeOutButton) intakeInButton = false;

        boolean footOutButton = gamepad2.a;
        boolean footUpButton = gamepad2.b;
        if (footOutButton && footUpButton) footOutButton = false;

        boolean catapultUpButton = gamepad2.right_bumper;
        boolean catapultDownButton = gamepad2.right_trigger > 0.2;
        boolean catapultDownReleased = gamepad2.right_trigger<0.2 && lastCata;
        lastCata = catapultDownButton;
        if (catapultUpButton && catapultDownButton) catapultUpButton = false;

        if (intakeInButton) {
            mRobot.intake.intake();
        } else if (intakeOutButton) {
            mRobot.intake.outtake();
        } else {
            mRobot.intake.stop();
        }

        if (footOutButton) {
            mRobot.foot.lower();
        } else if (footUpButton) {
            mRobot.foot.raise();
        } else {
            mRobot.foot.stop();
        }

        if (catapultUpButton) {
            mRobot.catapult.fire();
        } else if (catapultDownButton) {
            mRobot.catapult.load();
        }else if (catapultDownReleased){
            mRobot.catapult.hold();
        }

        if(gamepad1.start)
            mRobot.resetHeading();

        mRobot.drive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
              -  gamepad1.right_stick_x * 0.6
        );

        mRobot.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
