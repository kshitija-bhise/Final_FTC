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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//Blue Alliance Code
@Autonomous
@Configurable // Panels
public class RedAlliance extends OpMode {

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
        follower.setStartingPose(new Pose(110.00, 135.00, Math.toRadians(180)));


        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(110.000, 135.000), new Pose(89.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
//                align1 == 1
                .addPath(
                        new BezierLine(new Pose(89.000, 90.000), new Pose(89.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(89.000, 84.000), new Pose(127.000, 83.789))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //openGate1 == 3
                .addPath(
                        new BezierLine(new Pose(127.000, 83.789), new Pose(127.000, 74.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //openGate2 == 4
                .addPath(
                        new BezierLine(new Pose(127.000, 74.000), new Pose(130.526, 73.684))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot2 == 5
                .addPath(
                        new BezierLine(new Pose(130.526, 73.684), new Pose(89.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                //align2 == 6
                .addPath(
                        new BezierLine(new Pose(89.000, 90.000), new Pose(89.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //collect2 == 7
                .addPath(
                        new BezierLine(new Pose(89.000, 60.000), new Pose(128.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot3 == 8
                .addPath(
                        new BezierLine(new Pose(128.000, 60.000), new Pose(89.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                //align3 == 9
                .addPath(
                        new BezierLine(new Pose(89.000, 90.000), new Pose(89.000, 34.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //collect3 == 10
                .addPath(
                        new BezierLine(new Pose(89.000, 34.000), new Pose(126.000, 35.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot4 == 11
                .addPath(
                        new BezierLine(new Pose(126.000, 35.000), new Pose(82.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
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
//                cameraAlign.autoAlign();
//                Wait.mySleep(300);
//                acc.startNearShoot();
//                while(!acc.Goal()){
//                    acc.startNearShoot();
//                }
//                Wait.mySleep(1500);
//                acc.stopShooter();
//                acc.startIntake();
//                follower.setMaxPower(0.6);
                  Wait.mySleep(1500);

            }

            if(pathState == 3){
                acc.setLED(0.555);
                follower.setMaxPower(0.6);
            }

            if (pathState == 1 || pathState == 6 || pathState == 9){
//                acc.stopIntake();
                follower.setMaxPower(0.6);
            }

            if(pathState == 2 || pathState == 7 || pathState == 10) {
                acc.setLED(0.338);
                follower.setMaxPower(0.5);
                Wait.mySleep(1000);
            }

            if(pathState == 11){
//                cameraAlign.autoAlign();
//                acc.startFarShoot();
//                while(!acc.shoot()){
//                    acc.startFarShoot();
//                }
                acc.setLED(0.722);
                Wait.mySleep(1000);
//                acc.stopShooter();
//                follower.setMaxPower(0.6);
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