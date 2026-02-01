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

@Autonomous
@Configurable // Panels
public class RedFar12 extends OpMode {

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
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))

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

                // align == 4
                .addPath(
                        new BezierLine(new Pose(82.000, 18.000), new Pose(95.000, 57.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))

                //collect2 == 5
                .addPath(
                        new BezierLine(new Pose(95.000, 57.000), new Pose(130.000, 57.000)
                        )
                ).setConstantHeadingInterpolation(0)

                //shoot3 == 6
                .addPath(
                        new BezierLine(new Pose(130.000, 57.000), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(248))

                //align ==7
                .addPath(
                        new BezierLine(new Pose(82.000, 18.000), new Pose(89.000, 80.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(0))
                //Collect1 == 2
                .addPath(
                        new BezierLine(new Pose(89.000, 80.000), new Pose(128.500, 80.00))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                //shoot4 == 9
                .addPath(
                        new BezierLine(new Pose(128.000, 80.000), new Pose(82.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(248))

                //park == 10
                .addPath(
                        new BezierLine(new Pose(82.000, 18.000), new Pose(120.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(90))


//                .addPath(
//                        new BezierLine(new Pose(135.000, 57.000), new Pose(85.000, 85.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //collect3 == 7
//                .addPath(
//                        new BezierLine(new Pose(85.000, 85.000), new Pose(128.000, 80.00)
//                        )
//                ).setTangentHeadingInterpolation()
//
//                //shoot4 == 8
//                .addPath(
//                        new BezierLine(new Pose(128.000, 84.000), new Pose(85.000, 85.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                //openGate1 == 9
//                .addPath(
//                        new BezierLine(new Pose(85.000, 85.000), new Pose(120.000, 70.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))

                .build();

        follower.followPath(paths.getPath(pathState));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
                cameraAlign.autoAlign();
                Wait.mySleep(300);
                acc.startNearShoot();
                while(!acc.Goal()){
                    acc.startFarShoot();
                }
                Wait.mySleep(200);
                acc.startIntake();
                acc.stopShooter();
                follower.setMaxPower(0.9);
            }

            if(pathState == 1 || pathState == 4){
                follower.followPath(paths,false);
                follower.setMaxPower(0.9);
            }else {
                follower.setMaxPower(1.0);
            }

            if(pathState == 2 || pathState == 5 || pathState == 7){
                Wait.mySleep(300);
                acc.setLED(0.338);
                follower.setMaxPower(0.9);
            }else {
                follower.setMaxPower(1);
            }

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
