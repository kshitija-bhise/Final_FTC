package org.firstinspires.ftc.teamcode.Auto.RedAuto;

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
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//Blue Alliance Code
@Autonomous
@Configurable // Panels
public class RedNear12 extends OpMode {

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
                        new BezierLine(new Pose(110.000, 135.000), new Pose(90.00, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                //align1 == 1
                .addPath(
                        new BezierLine(new Pose(90.000, 90.000), new Pose(89.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(89.000, 80.000), new Pose(128.500, 80.00))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(128.500, 80.00), new Pose(90.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                //align2 == 4
                .addPath(
                        new BezierLine(new Pose(90.000, 90.000), new Pose(89.000, 55.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                //collect2 == 5
                .addPath(
                        new BezierLine(new Pose(89.000, 55.000), new Pose(128.000, 55.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot3 == 6
                .addPath(
                        new BezierLine(new Pose(128.000, 55.000), new Pose(90.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //align3 == 7
                .addPath(
                        new BezierLine(new Pose(90.000, 90.000), new Pose(89.000, 30.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                //collect3 == 8
                .addPath(
                        new BezierLine(new Pose(89.000, 30.000), new Pose(130.000, 30.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot3 == 9
                .addPath(
                        new BezierLine(new Pose(130.000, 30.000), new Pose(90.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //park
                .addPath(
                        new BezierLine(new Pose(90.000, 90.000), new Pose(120.000, 74.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(225),Math.toRadians(180))
                //align3 == 7
//                .addPath(
//                        new BezierLine(new Pose(79.000, 80.000), new Pose(89.000, 30.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
//                //collect3 == 8
//                .addPath(
//                        new BezierLine(new Pose(89.000, 30.000), new Pose(130.000, 30.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
                //shoot4 == 9
//                .addPath(
//                        new BezierLine(new Pose(130.000, 30.000), new Pose(82.000, 18.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
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
            if(pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9){
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startNearShoot();
                }
                Wait.mySleep(100);
                acc.stopShooter();
                acc.startIntake();
            }

//            if(pathState == 3){
//                acc.setLED(0.555);
//                follower.setMaxPower(0.5);
//            }
//
//            if(pathState == 4){
//                Wait.mySleep(200);
//                acc.setLED(0.555);
//                follower.setMaxPower(0.7);
//            }

            if (pathState == 1 || pathState == 4 || pathState == 7){
                follower.followPath(paths,false);
                follower.setMaxPower(0.8);
            }else{
                follower.setMaxPower(1);
            }

            if (pathState == 2 || pathState == 5 || pathState == 8){

            }

            pathState++;
            follower.followPath(paths.getPath(pathState));
        }
        Pose currentPose = follower.getPose();
        PoseMemory.savePose(currentPose);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}