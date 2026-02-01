package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
public class BlueNear18 extends OpMode {

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
        follower = Constants.createFollower(hardwareMap);
        aligner = new LimelightAligner(hardwareMap);
        follower.setStartingPose(new Pose(33.5, 137.0, Math.toRadians(0)));

        paths = follower
                .pathBuilder()
                //shoot == 0
                .addPath(
                        new BezierLine(new Pose(33.5, 137.0), new Pose(57, 86))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                //align2 == 1
                .addPath(
                        new BezierLine(new Pose(57, 86), new Pose(55.000, 64.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //collect2 == 2
                .addPath(
                        new BezierLine(new Pose(55.000, 64.000), new Pose(10.000, 64.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot3 == 3
                .addPath(
                        new BezierLine(new Pose(10.000, 64.000), new Pose(57, 86))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                //open gate == 4
                .addPath(
                        new BezierCurve(
                                new Pose(57, 86),
                                new Pose(54, 57),
                                new Pose(11, 50),
                                new Pose(9.5, 67)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(135))
                //collect == 5
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 67),
                                new Pose(10.5, 67)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))
                //shoot == 6
                .addPath(
                        new BezierCurve(
                                new Pose(10.5, 67),
                                new Pose(45, 67),
                                new Pose(57, 86)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(315))

                //open gate == 7
                .addPath(
                        new BezierCurve(
                                new Pose(57, 86),
                                new Pose(54, 57),
                                new Pose(11, 50),
                                new Pose(9.5, 67)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(135))
                //collect == 8
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 67),
                                new Pose(10.5, 67)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))
                //shoot == 9
                .addPath(
                        new BezierCurve(
                                new Pose(10.5, 67),
                                new Pose(45, 67),
                                new Pose(57, 86)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(315))

                //shoot1 == 10
                .addPath(
                        new BezierLine(new Pose(33.0, 137.0), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                //align1 == 11
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                //Collect1 == 12
                .addPath(
                        new BezierLine(new Pose(55.000, 90.000), new Pose(18.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //shoot2 == 13
                .addPath(
                        new BezierLine(new Pose(18.000, 90.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                //park == 14
                .addPath(
                        new BezierLine(
                                new Pose(58.000, 90.000),
                                new Pose(22.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
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
            if(pathState == -1){
                acc.rev();
            }

            if(pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9 || pathState == 13){
                acc.startNearShootAuto();
                while (!acc.Goal()){
                    acc.startNearShootAuto();
                }
                Wait.mySleep(200);
                acc.stopShooter();
                acc.startIntake();
            }

            if(pathState == 3){
                acc.startIntake();
            }

            if(pathState == 4 || pathState == 7){
                Wait.mySleep(1300);
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