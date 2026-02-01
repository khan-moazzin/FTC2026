package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Catapult;
import org.firstinspires.ftc.teamcode.subsystems.Foot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Robot {

    public Follower drive;

    public Intake intake;
    public Catapult catapult;
    public Foot foot;
    private IMU imu;
    private Telemetry telemetry;

    double imuOffset = 0;
    boolean robotOriented = false;
    public void init(HardwareMap hw, Telemetry tele) {
        this.telemetry = tele;

        // DRIVE
        drive = Constants.createFollower(hw);

        // SUBSYSTEMS
        intake = new Intake(hw);
        catapult = new Catapult(hw);
        foot = new Foot(hw);

        // ----------- IMU INITIALIZATION -----------
        imu = hw.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
        FtcDashboard.start(hw.appContext);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void update() {
        drive.update();
        catapult.update();
        sendTelemetry();
    }

    public void sendTelemetry() {
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Foot Power", foot.getPower());
        telemetry.addData("Catapult State", catapult.state);
        telemetry.addData("Catapult Seconds", catapult.timer.seconds());
        telemetry.addData("Heading", Math.toDegrees(getCurrentHeadingRadians()));
        telemetry.addData("Field Oriented", !robotOriented);
    }

    public void drive(double axial, double lateral, double yaw){
        double heading = getCurrentHeadingRadians();
        if(robotOriented)
            heading =0;


        drive.setTeleOpDrive(
                 axial,
                lateral,
                yaw,
                true,
                -heading
        );
    }
    public void resetHeading(){
        imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public double getCurrentHeadingRadians(){
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(imuOffset);
    }
    public void toggleRobotOriented(){
        this.robotOriented = !this.robotOriented;
    }
}
