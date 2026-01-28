package org.firstinspires.ftc.teamcode.Auto.AllianceAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable // Panels
public class RedBossBotixNear extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    CameraAlign cameraAlign;
    LimelightAligner aligner;


    @Override
    public void init() {
        acc  = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        follower.setStartingPose(new Pose(110.00, 135.00, Math.toRadians(180)));


        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(110.000, 135.000), new Pose(85, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                //align1 == 1
                .addPath(
                        new BezierLine(new Pose(85, 85), new Pose(89.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(89.000, 80.000), new Pose(128.500, 80.00))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(128, 80), new Pose(85, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //latch open == 4
                .addPath(
                        new BezierLine(new Pose(85, 85), new Pose(127, 76.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))

                //dummy shoot == 5
                .addPath(
                        new BezierLine(new Pose(127, 76.5), new Pose(90, 66))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))

                //align2 == 6
                .addPath(
                        new BezierLine(new Pose(90, 66), new Pose(89.000, 55.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))


                //collect2 == 7
                .addPath(
                        new BezierLine(new Pose(89.000, 55.000), new Pose(128.000, 55.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot == 8
                .addPath(
                        new BezierLine(new Pose(128, 55.000), new Pose(85, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //park
                .addPath(
                        new BezierLine(new Pose(85,85), new Pose(110, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                .build();

        // Build paths
        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if(pathState == 0 || pathState == 3 || pathState == 8){
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startNearShoot();
                }
                Wait.mySleep(100);
                acc.stopShooter();
                acc.startIntake();
            }

            if(pathState == 3){
                Wait.mySleep(1500);
            }

            if(pathState == 5){
                Wait.mySleep(800);
            }

            if (pathState == 1){
                follower.followPath(paths,false);
                follower.setMaxPower(0.8);
            }else{
                follower.setMaxPower(1);
            }

            if(pathState == 8){
                Wait.mySleep(500);
            }


            pathState++;
            follower.followPath(paths.getPath(pathState));
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}