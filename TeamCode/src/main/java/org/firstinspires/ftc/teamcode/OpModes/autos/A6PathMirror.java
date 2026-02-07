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

    public A6PathMirror(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.513, 122.293).mirror(), new Pose(43.607, 83.8697421981004).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180 -143.5))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.607, 83.8697421981004).mirror(), new Pose(5.544, 83.8697421981004).mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(5.544, 83.870).mirror(),
                        new Pose(22.022, 102.151).mirror(),
                        new Pose(21.513, 122.293).mirror()
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180-143.5))
                .build();
    }
}