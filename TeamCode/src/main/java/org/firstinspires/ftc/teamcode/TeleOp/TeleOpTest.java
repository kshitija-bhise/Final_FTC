package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class TeleOpTest extends LinearOpMode {
    Follower follower;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Limelight3A limelight;
    Acc acc;
    LimelightAligner aligner;
    public boolean shootfar;
    public boolean shootfardone = false;
    public boolean shootneardone = false;
    public PathChain pathsFar;
    public PathChain pathsNear;

    double desiredAngleDeg = 135;

    boolean isTeleOpDriveStarted = false;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.update();

        acc = new Acc(hardwareMap);

        follower.setStartingPose(new Pose(50, 25, Math.toRadians(0)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cameraAlign = new CameraAlign(hardwareMap);
        distanceEstimator = new DistanceEstimator(
                limelight,
                6.0,
                11.7,
                29.5
        );

        follower.startTeleopDrive(true);
        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean alignment = false;
            Pose pose = follower.getPose();
            double xRobo = follower.getPose().getX();
            double yRobo = follower.getPose().getY();
            double headingDeg = Math.toDegrees(pose.getHeading());
            if (gamepad1.left_trigger > 0.1) {
                alignment = true;
            } else {
                if (!isTeleOpDriveStarted) {
                    follower.pausePathFollowing();
                    follower.startTeleopDrive(true);
                    isTeleOpDriveStarted=true;
                }

            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            pathsFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(xRobo, yRobo), new Pose(58.000, -30.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headingDeg), Math.toRadians(315))
                    .build();
            pathsNear = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(xRobo, yRobo), new Pose(55.000, 23.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headingDeg), Math.toRadians(293))
                    .build();


            if (alignment && !shootneardone && !shootfardone) {
                double ShootNear = Math.sqrt((((xRobo - 58) * (xRobo - 58)) + ((yRobo + 30) * (yRobo + 30))));
                double ShootFar = Math.sqrt((((xRobo - 55) * (xRobo - 55)) + ((yRobo - 23) * (yRobo - 23))));
                if (ShootNear < ShootFar) {
                    shootfar = false;
                } else {
                    shootfar = true;
                }
                if (!shootfar && !(follower.atPose(new Pose(58, -30), 2, 2, Math.toRadians(2)))) {
                        follower.followPath(pathsFar);
                    shootfardone = true;

                }
                if (shootfar && !follower.atPose(new Pose(55, 23), 2, 2, Math.toRadians(2))) {
                        follower.followPath(pathsNear);
                    shootneardone = true;
                }
            }
            if (gamepad1.right_bumper && !isTeleOpDriveStarted) {
                telemetry.addData("Right bumper", "is pressed");
                follower.startTeleopDrive(true);
                follower.update();
            }


            telemetry.addData("Near", shootneardone);
            telemetry.addData("Far", shootfardone);
            telemetry.addData("alignment", alignment);
            telemetry.addData("X", xRobo);
            telemetry.addData("Y", yRobo);
            telemetry.addData("Heading", headingDeg);
            telemetry.update();

        }

    }
}


