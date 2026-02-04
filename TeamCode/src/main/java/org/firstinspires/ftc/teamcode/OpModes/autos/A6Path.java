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
    public PathChain Path4;

    public A6Path(Follower follower) {
        // Path1: starting to first target
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.513, 122.293), // start
                                new Pose(43.607, 59.693)   // end
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(0))
                .build();

        // Path2: intake approach
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(43.607, 59.693),
                                new Pose(20.544, 59.693)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // Path3: small curve to pickup or scoring position
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.544, 59.693),
                                new Pose(28.490, 70.546),
                                new Pose(15.505, 70.159)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path4: return to final scoring or starting area
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.505, 70.159),
                                new Pose(39.537, 108.921),
                                new Pose(21.900, 122.487)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143.5))
                .build();
    }
}
