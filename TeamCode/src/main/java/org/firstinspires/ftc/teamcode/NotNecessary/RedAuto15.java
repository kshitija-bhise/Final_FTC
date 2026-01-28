package org.firstinspires.ftc.teamcode.NotNecessary;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
@Configurable // Panels
public class RedAuto15 extends OpMode {

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
        follower.setStartingPose(new Pose(110.00, 135.00, Math.toRadians(180)));

        paths = follower
                .pathBuilder()

                //shoot near == 0
                .addPath(
                        new BezierLine(
                                new Pose(110.00, 135.00),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))

                //align == 1
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(89.000, 55.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                //collect == 2
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 55.000),

                                new Pose(125.000, 55.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                //open gate == 3
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 55.000),

                                new Pose(125.000, 65.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                //OG == 4
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 65.000),

                                new Pose(127.000, 65.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot near== 5
                .addPath(
                        new BezierLine(
                                new Pose(127.000, 65.000),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //align == 6
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(89.000, 80.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                //collect == 7
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 80.000),

                                new Pose(125.000, 80.000)
                        )
                ).setTangentHeadingInterpolation()

                //shoot == 8
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 80.000),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //align == 9
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(89.000, 30.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                //collect == 10
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 30.000),

                                new Pose(125.000, 30.000)
                        )
                ).setTangentHeadingInterpolation()

                //shoot == 11
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 30.000),

                                new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))

                //align == 12
                .addPath(
                        new BezierLine(
                                new Pose(82.000, 18.000),

                                new Pose(128.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))

                //collect == 13
                .addPath(
                        new BezierLine(
                                new Pose(128.000, 9.000),

                                new Pose(130.000, 9.000)
                        )
                ).setTangentHeadingInterpolation()

                //shoot far == 14
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 9.000),

                                new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))


                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if (pathState == 0 || pathState == 5 || pathState == 8) {
                acc.startNearShoot();
                Wait.mySleep(200);
                while(!acc.Goal()){
                    acc.startNearShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
                follower.setMaxPower(0.9);
            }

            if(pathState == 2 || pathState == 7 || pathState == 10 || pathState == 13){
                acc.stopIntake();
                acc.setLED(0.333);
                follower.setMaxPower(0.8);
            }

            if(pathState == 3 || pathState == 6 || pathState == 9 || pathState == 12){
                follower.followPath(paths.getPath(pathState), false);
                follower.setMaxPower(0.8);
            }

            if (pathState == 11 || pathState == 13){
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
                follower.setMaxPower(0.9);
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
