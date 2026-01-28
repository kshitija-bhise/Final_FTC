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
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
@Autonomous
@Configurable // Panels
public class RFZ extends OpMode {

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

                //collect == 1
                .addPath(
                        new BezierCurve(
                                new Pose(82.000, 18.000),
                                new Pose(79.789, 38.947),
                                new Pose(131.526, 33.000)
                        )
                ).setTangentHeadingInterpolation()


                //shoot == 2
                .addPath(
                        new BezierLine(
                                new Pose(131.526, 35.947),
                                new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))

                //collect == 3
                .addPath(
                        new BezierCurve(
                                new Pose(82.000, 18.000),
                                new Pose(81.000, 65.000),
                                new Pose(129.895, 57.000)
                        )
                ).setTangentHeadingInterpolation()

                //shoot == 4
                .addPath(
                        new BezierLine(
                                new Pose(130.737, 59.474),

                                new Pose(84.000, 82.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //collect == 5
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 82.000),

                                new Pose(128.316, 81.737)
                        )
                ).setTangentHeadingInterpolation()

                //shoot == 6
                .addPath(
                        new BezierLine(
                                new Pose(130.737, 59.474),

                                new Pose(84.000, 82.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if (pathState == 0 || pathState == 2){
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
                follower.setMaxPower(0.9);
            }

            if(pathState == 1 || pathState == 3 || pathState == 5){
                acc.slowIntake();
                Wait.mySleep(200);
                acc.stopIntake();
                acc.setLED(0.333);
                follower.setMaxPower(0.7);
            }

            if (pathState == 4 || pathState == 6){
                acc.startNearShoot();
                while (!acc.Goal()){
                    acc.startNearShoot();
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
