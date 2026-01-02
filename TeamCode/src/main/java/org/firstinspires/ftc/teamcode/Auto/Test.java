package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
//Blue Alliance Code
@Autonomous
@Configurable // Panels
public class Test extends OpMode {

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
        follower.setStartingPose(new Pose(33, 137.0, Math.toRadians(0)));


        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(33.0, 137.0), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                //align1 == 1
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 76.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 76.000), new Pose(20.000, 76.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //openGate1 == 3
                .addPath(
                        new BezierLine(new Pose(20.000, 76.000), new Pose(20.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                //openGate2 == 4
                .addPath(
                        new BezierLine(new Pose(20.000, 70.000), new Pose(9.500, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                //shoot2 == 5
                .addPath(
                        new BezierLine(new Pose(11.847, 70.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(325))
                //align2 == 6
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 52.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //collect2 == 7
                .addPath(
                        new BezierLine(new Pose(55.000, 52.000), new Pose(18.000, 52.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot3 == 8
                .addPath(
                        new BezierLine(new Pose(20.000, 52.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                //align3 == 9
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 28.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //collect3 == 10
                .addPath(
                        new BezierLine(new Pose(55.000, 28.000), new Pose(15.000, 28.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot4 == 11
                .addPath(
                        new BezierLine(new Pose(20.000, 28.000), new Pose(60.000, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))
                .setBrakingStart(0.1)
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
            if(pathState == 0 || pathState == 5 || pathState == 8){
                cameraAlign.autoTurn();
                Wait.mySleep(300);
                acc.startNearShoot();
                while(!acc.shoot()){
                    acc.startNearShoot();
                }
                Wait.mySleep(1500);
                acc.stopShooter();
                acc.startIntake();
            }

            if(pathState == 3){
                acc.setLED(0.555);
            }

            if (pathState == 2 || pathState == 7 || pathState == 10){
                acc.stopIntake();
                acc.setLED(0.338);
            }

            if(pathState == 11){
                cameraAlign.autoAlign();
                acc.startNearShoot();
                while(!acc.shoot()){
                    acc.startFarShoot();
                }
                Wait.mySleep(800);
                acc.stopShooter();
            }

            pathState++;
            follower.setMaxPower(0.8);
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