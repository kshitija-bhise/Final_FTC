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
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class BlueAllianceFar extends OpMode {

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
                        new BezierLine(new Pose(66.00, 15.000), new Pose(55.000, 38.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))
                //collect3 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 38.000), new Pose(17.000, 38.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(17.000, 38.000), new Pose(66.000, 18.00))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))

                //classifier == 4
                .addPath(
                        new BezierLine(new Pose(66, 18), new Pose(7, 43))
                ).setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(90))

                //collection = 5
                .addPath(
                        new BezierLine(new Pose(7,43),new Pose(7, 60))
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                //shoot = 6
                .addPath(
                        new BezierLine(new Pose(7, 60), new Pose(66, 18))
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(290))

                //park == 7
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(28, 77))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if (pathState == 0 || pathState == 3 || pathState == 6) {
                acc.startFarShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(100);
                acc.startIntake();
                acc.stopShooter();
            }

            if(pathState == 1 || pathState == 4){
                follower.followPath(paths,false);
            }

            if (pathState == 4){
                acc.startIntake();
            }

            if(pathState == 5){
                Wait.mySleep(1000);
            }

            pathState++;
            follower.followPath(paths.getPath(pathState));
        }
        // Log values to Panels and Driver Station

        Pose currentPose = follower.getPose();
        PoseMemory.savePose(currentPose);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}
