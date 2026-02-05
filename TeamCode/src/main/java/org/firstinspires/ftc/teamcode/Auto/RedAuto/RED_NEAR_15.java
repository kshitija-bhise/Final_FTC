package org.firstinspires.ftc.teamcode.Auto.RedAuto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Util.Wait;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RED_NEAR_15 extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState = 0; // Current autonomous path state (state machine)
    PathChain paths;
    Acc acc;
    CameraAlign cameraAlign;
    @Override
    public void runOpMode() {

        /* ================= INIT ================= */
        follower = Constants.createFollower(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        acc = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Pose startPose = new Pose(110.000, 135.000, Math.toRadians(180));
        follower.setStartingPose(startPose);

        acc.updateShooterState();

        /* ================= PATHS ================= */
        PathChain shoot1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(110.000, 135.000),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();

        PathChain align1 = new PathBuilder(follower)
                .addPath(
        new BezierLine(
                new Pose(85.000, 85.000),

                new Pose(85.000, 55.000)
        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        PathChain collect1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 55.000),

                                new Pose(135.000, 55.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        PathChain shoot2 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 55.000),
                                new Pose(100.000, 70.000),
                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
        PathChain latch = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(90.000, 45.000),
                                new Pose(135, 56)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(40))
                .build();
        PathChain comeback = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(135, 56),

                                new Pose(137, 56)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(40))
                .build();

        PathChain shoot3 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 55.000),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(225))
                .build();

        PathChain shoot4 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 55.000),
                                new Pose(110.000, 60.000),
                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(225))
                .build();

        PathChain align2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(80.000, 77.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225),Math.toRadians(0))
                .build();

        PathChain collect4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(80.000, 77.000),

                                new Pose(125.000, 77.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        PathChain shoot5 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(125.000, 77.000),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        PathChain park = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(120.000, 75.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90))
                .build();


        /* ================= START ================= */
        waitForStart();
        /* ================= AUTON SEQUENCE ================= */
        acc.AutoRev();
        runPath(shoot1, 1, 100);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(align1, 0.8, 0);
        runPath(collect1, 0.9, 100);
        acc.AutoRev();
        runPath(shoot2, 0.7, 200);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        runPath(latch, 0.9, 400);
        acc.startIntake();
        runPath(comeback, 0.9, 500);
        acc.AutoRev();
        runPath(shoot4, 0.7, 200);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        runPath(latch, 0.9, 400);
        acc.startIntake();
        runPath(comeback, 0.9, 500);
        acc.AutoRev();
        runPath(shoot4, 0.7, 200);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(collect4, 0.9, 100);
        acc.AutoRev();
        runPath(shoot5, 0.7, 200);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.stopShooter();
        runPath(park, 1, 0);
        /* ================= END ================= */

        Pose currentPose = follower.getPose();
        PoseMemory.savePose(currentPose);

    }



    /* ================= PATH RUNNER ================= */
    private void runPath(PathChain path, double power, long delayAfterMs) {
        follower.setMaxPower(power);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Velocity", acc.getShooterVelocity());
            telemetry.update();
        }

        if (delayAfterMs > 0) sleep(delayAfterMs);
    }





}
/* ================= END ================= */

