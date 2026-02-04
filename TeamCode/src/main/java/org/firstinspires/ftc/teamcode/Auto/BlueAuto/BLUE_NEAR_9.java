package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Mechanism.PoseMemory;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE_NEAR_9 extends LinearOpMode {
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
        Pose startPose = new Pose(34.000, 137.500, Math.toRadians(0));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */
        PathChain shoot1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(33.0, 137.0), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
        PathChain align1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        PathChain collect1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(55.000, 90.000), new Pose(18.000, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain shoot2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(18.000, 90.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        PathChain align2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(58.000, 90.000), new Pose(55.000, 64.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        PathChain collect2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(55.000, 64.000), new Pose(10.000, 64.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain shoot3 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(10.000, 64.000), new Pose(58.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();
        PathChain park = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(58.000, 90.000),
                                new Pose(22.000, 72.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        /* ================= START ================= */
        waitForStart();

        /* ================= AUTON SEQUENCE ================= */
        acc.AutoRev();
        runPath(shoot1, 1, 100);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(align1, 1, 0);
        runPath(collect1, 0.8, 100);
        acc.AutoRev();
        runPath(shoot2, 1, 100);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.startIntake();
        runPath(align2, 1, 0);
        runPath(collect2, 0.8, 100);
        acc.AutoRev();
        runPath(shoot3, 1, 100);
        acc.startNearShootAuto();
        acc.AutoContinousShoot();
        acc.stopShooter();
        acc.stopIntake();
        runPath(park, 1, 0);
        /* ================= END ================= */
        while (opModeIsActive()) {
        }
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

