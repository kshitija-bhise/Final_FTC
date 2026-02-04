package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name ="Pose Tracker")
public class PoseTracker extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

            Pose startingPose = new Pose(33.5, 137.0, Math.toRadians(0));
        follower.setPose(startingPose);

        telemetry.addLine("Pose Tracker Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.updatePose();

            Pose pose =  follower.getPose();

            double x = pose.getPose().getX();
            double y = pose.getPose().getY();
            double heading = pose.getPose().getHeading();
            double Deg = Math.toDegrees(heading);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", Deg);
            telemetry.update();
        }
    }
}
