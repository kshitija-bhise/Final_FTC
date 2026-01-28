package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import android.health.connect.datatypes.units.Velocity;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable // Panels
public class BlueNear9 extends OpMode {

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
        poseMemory = new PoseMemory();
        follower = Constants.createFollower(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        follower.setStartingPose(new Pose(33.0, 137.0, Math.toRadians(0)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(33.0, 137.0), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                //align1 == 1
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 90.000), new Pose(18.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(18.000, 90.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                //align2 == 4
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 64.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //collect2 == 5
                .addPath(
                        new BezierLine(new Pose(55.000, 64.000), new Pose(10.000, 64.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot3 == 6
                .addPath(
                        new BezierLine(new Pose(10.000, 64.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                //openGate1 == 9
                .addPath(
                        new BezierLine(
                                new Pose(58.000, 90.000),
                                new Pose(22.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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
            if(pathState == 0 || pathState == 3 || pathState == 6){
                acc.startNearShoot();
                while (!acc.Goal()) {
                    acc.startNearShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
            }

            if (pathState == 1 || pathState == 4 || pathState == 7){
                follower.followPath(paths,false);
                follower.setMaxPower(0.8);
            }else {
                follower.setMaxPower(1);
            }

            if (pathState == 2 || pathState == 5 || pathState == 8){
                acc.slowIntake();
                acc.setLED(0.338);
            }
            if (pathState == 9){
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