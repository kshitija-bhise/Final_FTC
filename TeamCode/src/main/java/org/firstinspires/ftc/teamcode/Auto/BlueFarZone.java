package org.firstinspires.ftc.teamcode.Auto;

import static android.os.SystemClock.sleep;

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
public class BlueFarZone extends OpMode {

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
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                    //shoot1 == 0
                    .addPath(
                            new BezierLine(new Pose(56, 8), new Pose(60, 15))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(290))
                    //align==1
                    .addPath(
                            new BezierLine(new Pose(60.000, 15.000), new Pose(48.000, 28.00))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(180))
                    //collect1 == 2
                    .addPath(
                            new BezierLine(new Pose(48.000, 28.000), new Pose(14.000, 35.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    //shoot2 == 3
                    .addPath(
                            new BezierLine(new Pose(13.000, 35.000), new Pose(56.000, 21.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(285))
//                    //align2 == 4
                    .addPath(
                            new BezierLine(new Pose(56.000, 21.000), new Pose(45.000, 54.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(285), Math.toRadians(180))
//                    //collect2 == 5
                    .addPath(
                            new BezierLine(new Pose(45.000, 54.000), new Pose(16.632, 59.579))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    //open gate == 6
                    .addPath(
                            new BezierLine(new Pose(16.632, 59.579), new Pose(17.000, 65.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    //gate open == 7
                    .addPath(
                            new BezierLine(new Pose(17.000, 65.000), new Pose(12.000, 65.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    //shoot3 == 8
                    .addPath(
                            new BezierLine(new Pose(15.000, 65.000), new Pose(59.579, 84.842))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    //align3 == 9
                    .addPath(
                            new BezierLine(new Pose(59.579, 84.842), new Pose(37.474, 81.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    //collect3 == 10
                    .addPath(
                            new BezierLine(new Pose(37.474, 81.000), new Pose(15.158, 83.789))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    //shoot4 == 11
                    .addPath(
                            new BezierLine(new Pose(15.158, 83.789), new Pose(60.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    // parek
                .addPath(
                        new BezierLine(new Pose(60.000, 85.000), new Pose(60.000, 75.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
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
            if( pathState == 8 || pathState == 11 ){
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startNearShoot();
                }
                Wait.mySleep(500);
                acc.stopShooter();
            }
            if( pathState == 6 || pathState == 7){
                follower.setMaxPower(0.6);

            }

            if(pathState == 2 || pathState == 5|| pathState == 10){acc.stopIntake();}

            if(pathState == 12){
                follower.setMaxPower(0.6);
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
