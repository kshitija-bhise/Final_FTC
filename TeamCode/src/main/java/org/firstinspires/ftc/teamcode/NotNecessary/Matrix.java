package org.firstinspires.ftc.teamcode.NotNecessary;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
@Configurable // Panels
public class Matrix extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    LimelightAligner aligner;
    PoseMemory poseMemory;

    @Override
    public void init() {
        acc  = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(62.000, 8.000, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(62.000, 8.000), new Pose(66.000, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(290))
                //align==1
                .addPath(
                        new BezierLine(new Pose(66.00, 15.000), new Pose(55.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))
                //collect3 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 36.000), new Pose(15.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
//              //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(15.000, 36.000), new Pose(66.000, 18.00))
                )
                //align==1
                .addPath(
                        new BezierLine(new Pose(66.00, 15.000), new Pose(9.000, 9.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(190))

                //collect3 == 8
                .addPath(
                        new BezierLine(new Pose(9,9), new Pose(20.000, 5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(190))
                //collect3 == 9
                .addPath(
                        new BezierLine(new Pose(20,5), new Pose(9,5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(190))
//                   //shoot2 == 10
                .addPath(
                        new BezierLine(new Pose(9,5), new Pose(66.000, 18.00))
                )
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(290))
                //park == 11
                .addPath(
                        new BezierLine(new Pose(66.000, 18.00), new Pose(20,20))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(270))
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
            if(pathState == 0 || pathState == 3 || pathState == 7){
                acc.startFarShoot();
                while (!acc.Goal()) {
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
            }

            if (pathState == 1 || pathState == 5){
                follower.followPath(paths,false);
                follower.setMaxPower(0.8);
            }else {
                follower.setMaxPower(1);
            }

            if (pathState == 2 || pathState == 6 ){
                acc.slowIntake();
            }
            if (pathState == 8){
                acc.stopShooter();
            }
            pathState++;
            follower.followPath(paths.getPath(pathState));
        }

        Pose currentPose = follower.getPose();
        PoseMemory.savePose(currentPose);


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Velocity", acc.getShooterVelocity());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}