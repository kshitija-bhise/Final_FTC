package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class SemiAutoRed extends OpMode {
    private Follower follower;
    PoseMemory poseMemory;

    Limelight3A limelight;
    LimelightAligner aligner;
    Acc acc;
    public  static Pose startPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathPark;
    private Supplier<PathChain> pathFar;
    private Supplier<PathChain> pathNear;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        telemetry.addLine("Last Pose: ");
        PoseMemory.printLastPose(telemetry);
        if(PoseMemory.hasPose()){
            startPose = new Pose(
                    PoseMemory.getLastX(),
                    PoseMemory.getLastY(),
                    PoseMemory.getLastHeading()
            );
        }else {
            startPose = new Pose(110, 135, Math.toRadians(180));
        }


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        acc = new Acc(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower.startTeleopDrive(true);
        Servo lock = hardwareMap.get(Servo.class,"lock");
        lock.setPosition(0);
        limelight.pipelineSwitch(1);
        follower.update();

        poseMemory = new PoseMemory();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathPark = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(40, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();


        pathNear = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(90, 90))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();

        pathFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(82, 18))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(248), 0.8))
                .build();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }


        //Automated PathFollowing
        if (gamepad1.crossWasPressed()) {
            follower.followPath(pathPark.get()); //Lazy Curve Generation
            automatedDrive = true;
        }

        if (gamepad1.squareWasPressed()) {
            follower.followPath(pathFar.get());
            automatedDrive = true;
        }

        if (gamepad1.triangleWasPressed()) {
            follower.followPath(pathNear.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.circleWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * 0.95,
                -gamepad1.left_stick_x * 0.95,
                -gamepad1.right_stick_x * 0.6,
                true
        );
        follower.update();

        if(gamepad2.right_trigger > 0.5) {
            acc.ContinousShoot();
        }
        else if (gamepad2.a){
            acc.startNearShoot();
            acc.rev();
        } else if (gamepad2.y){
            acc.startFarShoot();
            acc.revfar();
        } else {
            acc.stopShooter();
        }


        if(gamepad2.left_bumper){
            acc.startIntake();
        } else if (gamepad2.b) {
            acc.OutTake();
        }else if(gamepad2.right_bumper) {
            acc.TriggerShoot();
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
        }
        PoseMemory.printLastPose(telemetry);
        telemetryM.debug("position", follower.getPose());
        telemetry.addData("X coordinate", follower.getPose().getX());
        telemetry.addData("Y coordinate", follower.getPose().getY());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("power", follower.getMaxPowerScaling());
        telemetry.update();
    }
}