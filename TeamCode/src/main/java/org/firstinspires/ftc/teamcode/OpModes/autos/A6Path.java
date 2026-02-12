package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// -----------------------------------------------------
// PATH DEFINITIONS
// -----------------------------------------------------
public class A6Path {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain leave;

    public A6Path(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.513, 122.293), new Pose(43.607, 83.8697421981004))
                )
                .setConstantHeadingInterpolation(Math.toRadians(143.5))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.607, 83.8697421981004), new Pose(5.544, 83.8697421981004))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3  = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(5.544, 83.870),
                                new Pose(38.630, 100.978),
                                new Pose(21.318, 125.810)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))

                .build();

        leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.778, 124.247),

                                new Pose(36.342, 139.897)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(143.5))

                .build();
    }
}