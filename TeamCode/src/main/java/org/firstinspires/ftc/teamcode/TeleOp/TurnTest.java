package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "TURNTEST")
public class TurnTest extends LinearOpMode {

    Follower follower;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Limelight3A limelight;

    double desiredAngleDeg = 135;

    boolean isTeleOpDriveStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.update();

        follower.setStartingPose(new Pose(0, 0, Math.toRadians(180)));

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

        while (opModeIsActive()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            Pose pose = follower.getPose();

            double headingDeg = Math.toDegrees(pose.getHeading());
            double error = desiredAngleDeg - headingDeg;

            if (gamepad1.bWasPressed()) {
                follower.followPath(BuildPath.getTurnPath(follower.getPose(), Math.abs(error), error < 0));
                isTeleOpDriveStarted = false;
            } else if (gamepad1.b) {
                if ((!follower.isTurning() && !follower.isBusy()) && !isTeleOpDriveStarted) {
                    follower.startTeleopDrive(true);
                    isTeleOpDriveStarted = true;
                }
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, cameraAlign.alignTurnValue(gamepad1.right_stick_x), true);
            } else {
                boolean isGamePadActive = Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.02;
                if (((!follower.isTurning() && !follower.isBusy()) || isGamePadActive) && !isTeleOpDriveStarted) {
                    follower.startTeleopDrive(true);
                    isTeleOpDriveStarted = true;
                }
            }
        }
    }

}
