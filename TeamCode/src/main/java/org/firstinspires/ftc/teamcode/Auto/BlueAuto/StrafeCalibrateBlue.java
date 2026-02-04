package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Strafe Calibrate - Blue")
public class StrafeCalibrateBlue extends LinearOpMode {

    public static double TEST_POWER = 0.95;          // keep smooth
    public static double TEST_LATERAL = 1.30;        // change this number

    Follower follower;

    @Override
    public void runOpMode() {
        Constants.LATERAL_MULTIPLIER = TEST_LATERAL;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34.0, 137.5, Math.toRadians(0)));

        PathChain strafe = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(34.0, 137.5),
                        new Pose(34.0, 97.5)     // 40 inch strafe (change if you want)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        waitForStart();

        follower.setMaxPower(TEST_POWER);
        follower.followPath(strafe);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("LATERAL_MULTIPLIER", TEST_LATERAL);
            telemetry.update();
        }
    }
}
