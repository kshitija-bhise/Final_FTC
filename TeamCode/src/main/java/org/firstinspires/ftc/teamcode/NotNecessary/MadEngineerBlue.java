package org.firstinspires.ftc.teamcode.NotNecessary;

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
public class MadEngineerBlue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    LimelightAligner aligner;
    CameraAlign cameraAlign;

    @Override
    public void init() {
        acc  = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        follower.setStartingPose(new Pose(62.000, 8.000, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(62.000, 8.000), new Pose(62.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(290))

                //park == 7
                .addPath(
                        new BezierLine(
                                new Pose(62,15),

                                new Pose(36,12)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))

                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if (pathState == 0 ) {
                acc.startFarShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.startIntake();
                acc.stopShooter();
                follower.setMaxPower(0.9);}



//            if(pathState == 6 || pathState == 8){
//                acc.startNearShoot();
//                while(!acc.Goal()){
//                    acc.startNearShoot();
//                }
//                Wait.mySleep(200);
//                acc.stopShooter();
//                acc.startIntake();
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
