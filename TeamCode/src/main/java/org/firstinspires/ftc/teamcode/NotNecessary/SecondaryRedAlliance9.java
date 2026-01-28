package org.firstinspires.ftc.teamcode.NotNecessary;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
public class SecondaryRedAlliance9 extends OpMode {

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
        follower.setStartingPose(new Pose(82.000, 8.000, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(
                                new Pose(82.000, 8.000),

                                new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))

                //Collect pt1 == 1
                .addPath(
                        new BezierLine(
                                new Pose(82.000, 18.000),

                                new Pose(135.000, 10.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))

                //align1 == 2
                .addPath(
                        new BezierCurve(
                                new Pose(135.000, 10.0 ),
                                new Pose(125.000, 9.000),
                                new Pose(120.000, 5.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                //Collect pt2 == 3
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 5.500),

                                new Pose(135.000, 5.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot2 == 4
                .addPath( new BezierLine(
                                new Pose(135.000, 5.500),

                                new Pose(82.000,18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))

                // collect2 == 5
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 20),

                                new Pose(135.000, 20.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(350))

                //shoot3 == 6
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 20.000),

                                new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(250))

                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            acc.rev();
            if(pathState == 0 || pathState == 4 || pathState == 6) {
                acc.startFarShoot();
                while (!acc.Goal()){
                    acc.startFarShoot();
                }
                acc.startIntake();
                follower.setMaxPower(0.9);
            }

            if(pathState == 1){
                Wait.mySleep(900);
            }

            if (pathState == 3){
                acc.slowIntake();
                acc.setLED(0.611);
                acc.rev();
                follower.setMaxPower(1.0);
            }

            if (pathState == 5){
                Wait.mySleep(8000);
                acc.slowIntake();
                acc.setLED(0.611);
                acc.rev();
                follower.setMaxPower(1.0);
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
