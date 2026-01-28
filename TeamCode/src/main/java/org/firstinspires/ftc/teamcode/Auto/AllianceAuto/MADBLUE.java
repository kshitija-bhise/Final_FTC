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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class MADBLUE extends OpMode {

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
//                   //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(15.000, 36.000), new Pose(66.000, 18.00))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))
                   //align2 == 4
                .addPath(
                        new BezierLine(new Pose(66.000, 18.000), new Pose(9.0, 13.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(190))
//               //shoot3 == 5
                .addPath(
                        new BezierLine(new Pose(9.0, 13.5), new Pose(62, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(295))
                //collect == 6
                .addPath(
                        new BezierLine(new Pose(62, 15), new Pose(9, 8.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(190))
                //shoot == 7
                .addPath(
                        new BezierLine(new Pose(9.0, 8.5), new Pose(62, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(295))
                // park == 8
                .addPath(
                        new BezierLine(new Pose(62.000, 15.000), new Pose(30, 15))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(90))
                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if(pathState == 0 || pathState == 3 || pathState == 5 || pathState == 7){
                cameraAlign.autoAlign();
                Wait.mySleep(300);
                acc.startFarShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
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
