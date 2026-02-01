package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red Alliance TeleOp")
public class RedAllianceTele extends LinearOpMode {

    Follower follower;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Limelight3A limelight;
    Acc acc;
    LimelightAligner aligner;
    TelemetryManager panelsTelemetry;

    double desiredAngleDeg = 225;

    boolean isTeleOpDriveStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.update();

        acc = new Acc(hardwareMap);

        aligner = new LimelightAligner(hardwareMap);


        follower.setStartingPose(new Pose(0, 0, Math.toRadians(180)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cameraAlign = new CameraAlign(hardwareMap);
        distanceEstimator = new DistanceEstimator(
                limelight,
                0.0,
                15.5,
                29.5
        );

        follower.startTeleopDrive(true);
        Servo lock = hardwareMap.get(Servo.class,"lock");
        lock.setPosition(0);
        limelight.pipelineSwitch(1);
        follower.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            Pose pose = follower.getPose();
            double headingDeg = Math.toDegrees(pose.getHeading());
            double error = desiredAngleDeg - headingDeg;

            if(gamepad2.right_trigger > 0.1){
                acc.startNearShoot();
                acc.ContinousShoot();
            } else if (gamepad2.a){
                acc.rev();
            } else if (gamepad2.y){
                acc.revfar();
            } else if(gamepad2.left_trigger > 0.1){
                acc.startFarShoot();
                acc.shootThree();
            }else {
                acc.stopShooter();
            }

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.95,
                    -gamepad1.left_stick_x * 0.95,
                    -gamepad1.right_stick_x * 0.6,
                    true
            );

            if(gamepad1.y){
                lock.setPosition(0.2);
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
            telemetry.addData("Velo", acc.getShooterVelocity());
            telemetry.addData("Heading", headingDeg);
            telemetry.update();

        }
    }

}
