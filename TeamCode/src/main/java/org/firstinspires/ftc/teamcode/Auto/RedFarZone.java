package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class RedFarZone extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    CameraAlign cameraAlign;

    @Override
    public void init() {
        acc  = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        follower.setStartingPose(new Pose(83.000, 8.000, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(83.000, 8.000), new Pose(83.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))
                //align==1
                .addPath(
                        new BezierLine(new Pose(83.000, 21.000), new Pose(89.684, 35.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
                //collect1 == 2
                .addPath(
                        new BezierLine(new Pose(89.684, 35.579), new Pose(125.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(125.000, 36.000), new Pose(83.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
                //align2 == 4
                .addPath(
                        new BezierLine(new Pose(83.000, 21.000), new Pose(90.000, 59.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
                //collect2 == 5
                .addPath(
                new BezierLine(new Pose(90.000, 59.000), new Pose(124.000, 59.000))
        )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //open gate == 6
                .addPath(
                        new BezierLine(new Pose(124.000, 59.000), new Pose(124.000, 64.632))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                //gate open == 7
                .addPath(
                        new BezierLine(new Pose(124.000, 64.632), new Pose(129.263, 64.632))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                 //shoot3 == 8
                .addPath(
                        new BezierLine(new Pose(129.263, 64.632), new Pose(89.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                //align3 == 9
                .addPath(
                        new BezierLine(new Pose(89.000, 90.000), new Pose(89.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                //collect3 == 10
                .addPath(
                        new BezierLine(new Pose(89.000, 84.000), new Pose(125.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot4 == 11
                .addPath(
                        new BezierLine(new Pose(125.000, 84.000), new Pose(89.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
//                //parks == 12
//                .addPath(
//                        new BezierLine(new Pose(89.000, 90.000), new Pose(89.000, 80.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if(pathState == 0 || pathState == 3){
                cameraAlign.autoAlign();
                acc.startFarShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(500);
                acc.stopShooter();
                acc.startIntake();
                follower.setMaxPower(0.6);
            }
//            if( pathState == 8 || pathState == 11 ){
//                acc.startNearShoot();
//                while(!acc.Goal()){
//                    acc.startNearShoot();
//                }
//                Wait.mySleep(500);
//                acc.stopShooter();
//                follower.setMaxPower(0.6);
//            }
//            if( pathState == 6 || pathState == 7){
//                follower.setMaxPower(0.6);
//
//            }
//
//            if(pathState == 2 || pathState == 5|| pathState == 10){acc.stopIntake();
//                follower.setMaxPower(0.6);}
//
//            if(pathState == 12){
//                follower.setMaxPower(0.6);
//            }

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
