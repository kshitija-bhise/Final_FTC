package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@TeleOp
public class teleoptest2 extends LinearOpMode {
    ElapsedTime timer;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Follower follower;
    boolean shoot;
    Acc acc;
    boolean isTeleOpDriveStarted = false;
    private boolean isRobotCentric = true;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    Servo lock;
    LimelightAligner aligner;

    @Override
    public void runOpMode() throws InterruptedException {
        acc = new Acc(hardwareMap);
        timer = new ElapsedTime();
        cameraAlign = new CameraAlign(hardwareMap);
        distanceEstimator = new DistanceEstimator(hardwareMap.get(Limelight3A.class, "limelight"),
                6,
                11.7,
                29.5);
        follower = Constants.createFollower(hardwareMap);
        shoot = false;
        lock = hardwareMap.get(Servo.class,"lock");
        aligner = new LimelightAligner(hardwareMap);

        lock.setPosition(0.15);

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        follower.startTeleopDrive(true);
        follower.update();
        while (opModeIsActive()) {
            double driveY = -gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if(gamepad1.left_bumper){
                slowModeMultiplier = 0.5;
            }else{
                slowModeMultiplier = 1;
            }


            if(gamepad1.y){
                lock.setPosition(0.27);
            }

            if(gamepad2.right_trigger > 0.1){
                while (gamepad2.right_trigger > 0.1){
                    acc.startNearShoot();
                    acc.Goal();
                }
            } else if (gamepad2.y){
                acc.startNearShoot();
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
                if (aligner.isAligned()) {
                    aligner.stopMotors();
                    acc.setLED(0.388);
                }
                else {
                    aligner.alignToAprilTag();
                }
                acc.setLED(0.277);
                telemetry.addData("Ty", distanceEstimator.getTy());
            }
            else {
                acc.setLED(0.0); // default state
            }

            follower.setTeleOpDrive(driveY, driveX, turn, true);
            follower.update();
            display();
        }
    }

    public void display() {
        telemetry.addData("Distance", distanceEstimator.getDistanceInches());
        telemetry.addData("Shoot", shoot);
        telemetry.addData("Velocity", acc.getShooterVelocity());
        telemetry.addData("Tx", distanceEstimator.getTx());
        telemetry.addData("Pose", Math.toDegrees(follower.getHeading()));
        telemetry.addData("TeleOp", isTeleOpDriveStarted);
        telemetry.update();
    }
}