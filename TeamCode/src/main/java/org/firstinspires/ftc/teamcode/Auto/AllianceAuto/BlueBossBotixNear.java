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
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable // Panels
public class BlueBossBotixNear extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    LimelightAligner aligner;

    @Override
    public void init() {
        acc  = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        follower.setStartingPose(new Pose(34.0, 136.0, Math.toRadians(0)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(34.000, 135.000), new Pose(54.00, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                //align1 == 1
                .addPath(
                        new BezierLine(new Pose(54.000, 90.000), new Pose(55.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 90.000), new Pose(17.500, 90.00))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(17.500, 90.00), new Pose(54.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                //align2 == 4
                .addPath(
                        new BezierLine(new Pose(54.000, 90.000), new Pose(55.000, 64.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //collect2 == 5
                .addPath(
                        new BezierLine(new Pose(55.000, 64.000), new Pose(12, 64.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //openGate1 == 6
                .addPath(
                        new BezierLine(new Pose(12, 64), new Pose(24.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                //openGate2 == 7
                .addPath(
                        new BezierLine(new Pose(24.000, 70.000), new Pose(15.000, 70.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                //shoot3 == 8
                .addPath(
                        new BezierLine(new Pose(15.000, 74.000), new Pose(54.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                //Classifier end == 9
                .addPath(
                        new BezierLine(new Pose(54.000, 90.000), new Pose(20.000, 74.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315),Math.toRadians(270))
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

                if(pathState == 5){
                    Wait.mySleep(800);
                }

                if (pathState == 1){
                    follower.followPath(paths,false);
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