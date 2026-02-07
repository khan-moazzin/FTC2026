package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

public class Constants {
    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    public static double INTAKE_IN_POWER = 1.0;
    public static double INTAKE_OUT_POWER = -0.9;
    public static double INTAKE_OFF_POWER = 0.0;

    public static double FOOT_UP_POWER = 1.0;
    public static double FOOT_DOWN_POWER = -0.85;
    public static double FOOT_OFF_POWER = 0.0;

    public static double CATAPULT_UP_POWER = -1.0;
    public static double CATAPULT_DOWN_POWER = 1.0;
    public static double CATAPULT_HOLD_POWER = 0.2;

    public static  double FIRE_TIME = 1;
    public static  double LOAD_TIME = 1.5;


    public static FollowerConstants followerConstants = new FollowerConstants().mass(11.6)
            .translationalPIDFCoefficients(new PIDFCoefficients(1,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(12,0,0,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.00001,
                    0.6,
                    0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.8, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap, Telemetry telemetry) {//todo tuning
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(
//                        new FusedLocalizer(
//                        new DriveEncoderLocalizer(hardwareMap, localizerConstants),
//                        new LimelightVision(hardwareMap),
//                                telemetry
//                )
                        new DriveEncoderLocalizer(hardwareMap,localizerConstants)
                )
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(56.354)
            .yVelocity(58.5402);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .robotLength(10)
            .robotWidth(15.75)
            .forwardTicksToInches(.00828578*2/3)
            .strafeTicksToInches(.006686)
            .turnTicksToInches(0.01251);
}
