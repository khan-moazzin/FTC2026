package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// -----------------------------------------------------
// PATH DEFINITIONS
// -----------------------------------------------------
public class A6PathMirror {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;

    public A6PathMirror(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.513, 122.293).mirror(), new Pose(43.607, 59.693).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180 + 143.5), Math.toRadians(0))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.607, 59.693).mirror(), new Pose(20.544, 59.693).mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.544, 59.693).mirror(),
                                new Pose(28.490, 70.546).mirror(),
                                new Pose(15.505, 70.159).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.505, 70.159).mirror(),
                                new Pose(39.537, 108.921).mirror(),
                                new Pose(21.900, 122.487).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180+143.5))
                .build();
    }
}