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
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class MADRED extends OpMode {

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
        follower.setStartingPose(new Pose(82.000, 8.000, Math.toRadians(270)));

        paths = follower
                .pathBuilder()
                //shoot1 == 0
                .addPath(
                        new BezierLine(new Pose(82.000, 8.000), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(248))

                //align == 1
                .addPath(
                        new BezierLine(new Pose(82.000, 18.000), new Pose(95.000, 33.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(0))

                //collect1 == 2
                .addPath(
                        new BezierLine(new Pose(95.000, 33.000), new Pose(130.000, 33.000)
                        )
                ).setTangentHeadingInterpolation()

                //shoot2 == 3
                .addPath(
                        new BezierLine(new Pose(130.000, 33.000), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(248))

                //collect == 4
                .addPath(
                        new BezierLine(new Pose(82, 18), new Pose(135, 13.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(350))

                //shoot2 == 5
                .addPath(
                        new BezierLine(new Pose(135.000, 13.5), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(248))

                //collect == 6
                .addPath(
                        new BezierLine(new Pose(82.00, 18.0), new Pose(135.00, 8.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(350))

                //shoot2 == 7
                .addPath(
                        new BezierLine(new Pose(135, 8.5), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(248))


                //park == 8
                .addPath(
                        new BezierLine(new Pose(82.000, 18.000), new Pose(108, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(0))


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

            if(pathState == 1){
                follower.followPath(paths,false);
                follower.setMaxPower(0.8);
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
