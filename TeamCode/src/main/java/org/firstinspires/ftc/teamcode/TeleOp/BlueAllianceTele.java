package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Blue Alliance TeleOp")
public class BlueAllianceTele extends LinearOpMode {

    Follower follower;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Limelight3A limelight;
    Acc acc;
    LimelightAligner aligner;

    double desiredAngleDeg = 135;

    boolean isTeleOpDriveStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.update();

        acc = new Acc(hardwareMap);

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

        waitForStart();

        while (opModeIsActive()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            Pose pose = follower.getPose();

            double headingDeg = Math.toDegrees(pose.getHeading());
            double error = desiredAngleDeg - headingDeg;

            if(gamepad2.right_trigger > 0.1){
                while (gamepad2.right_trigger > 0.1){
                    acc.startNearShoot();
                    acc.Goal();
                }
            } else if (gamepad2.y){
                acc.rev();
            } else if(gamepad2.left_trigger > 0.1){
                while(gamepad2.left_trigger > 0.1){
                    acc.startFarShoot();
                    acc.Goal();
                }
            }else {
                acc.stopShooter();
            }

            if(gamepad2.left_bumper){
                acc.startIntake();
            } else if (gamepad2.right_bumper) {
                acc.OutTake();
            } else{
                acc.stopIntake();
            }

            if (gamepad1.right_bumper) {
                telemetry.addData("Pipeline Number",limelight.getLatestResult().getPipelineIndex());
                if (!aligner.isAligned()) {
                    aligner.alignToAprilTag();
                }
                else {
                    aligner.stopMotors();
                    acc.setLED(0.611);
                    gamepad1.rumble(300);
                    gamepad2.rumble(300);
                }
                acc.setLED(0.277);
                telemetry.addData("Ty", distanceEstimator.getTy());
            }
            else {
                acc.setLED(0.0); // default state
            }


            if (gamepad1.left_bumper) {
                follower.followPath(BuildPath.getTurnPath(follower.getPose(), Math.abs(error), error < 0));
                isTeleOpDriveStarted = false;
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
