package org.firstinspires.ftc.teamcode.Auto.RedAuto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RED_FAR_12 extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public Follower follower; // Pedro Pathing follower instance
    Acc acc;
    CameraAlign cameraAlign;

    @Override
    public void runOpMode() {

        /* ================= INIT ================= */
        follower = Constants.createFollower(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        acc = new Acc(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Pose startPose = new Pose(82.000, 8.000, Math.toRadians(270));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */
        PathChain shoot1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(82.000, 8.000), new Pose(83.500, 23.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                .build();

        PathChain align1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(83.500, 23.000), new Pose(95.000, 33.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();

        PathChain collect1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(95.000, 33.000), new Pose(130.000, 33.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        PathChain shoot2 = new PathBuilder(follower)
                .addPath(
                new BezierLine(new Pose(130.000, 33.000), new Pose(83.500, 23.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                .build();

        PathChain align2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(83.500, 23.000), new Pose(95.000, 57.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();

        PathChain collect2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(95.000, 57.000), new Pose(130.000, 57.000)
                        )
                ).setConstantHeadingInterpolation(0)
                .build();

        PathChain shoot3 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(130.000, 57.000), new Pose(83.500, 23.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                .build();
        
        PathChain collect3 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(83.500, 23.000), new Pose(135, 12.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(350))
                .build();

        PathChain shoot4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(135.000, 12.5), new Pose(83.500, 23.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(245))
                .build();
        
        PathChain collect4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(83.500, 23.000), new Pose(135.00, 6.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(355))
                .build();
        
        PathChain shoot5 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(135, 6.5), new Pose(83.500, 23.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(355), Math.toRadians(245))
                .build();
        PathChain park = new PathBuilder(follower)
                .addPath(
        new BezierLine(new Pose(83.500, 23.000), new Pose(108, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();

        /* ================= START ================= */
        waitForStart();

        /* ================= AUTON SEQUENCE ================= */
        acc.AutoRevFar();
        runPath(shoot1, 0.6, 700);
        acc.startFarShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(align1, 1, 0);
        runPath(collect1, 0.7, 100);
        acc.AutoRevFar();
        runPath(shoot2, 0.8, 200);
        acc.startFarShootAuto();;
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(align2, 1, 0);
        runPath(collect2, 0.7, 100);
        acc.AutoRevFar();
        runPath(shoot3, 0.8, 500);
        acc.startFarShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(collect3, 1, 500);
        acc.AutoRevFar();
        runPath(shoot4, 0.8, 500);
        acc.startFarShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(collect4, 1, 500);
        acc.AutoRevFar();
        runPath(shoot5, 0.8, 500);
        acc.startFarShootAuto();
        acc.AutoContinousShoot();
        acc.stopShooter();
        acc.stopIntake();
        runPath(park, 0.8, 100);
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
        }

        if (delayAfterMs > 0) sleep(delayAfterMs);
    }

}
/* ================= END ================= */

