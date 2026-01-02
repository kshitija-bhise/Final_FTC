package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class DistanceTest extends OpMode {
    private DistanceEstimator distanceEstimator;
    private Follower follower;
    private Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        distanceEstimator = new DistanceEstimator(limelight, 6.0, 11.7, 29.5);

    }

    @Override
    public void loop() {
        if (!distanceEstimator.hasTarget()) {
            telemetry.addData("Status", "No Target");
            stop();
            telemetry.update();
            return;
        }

        double distance = distanceEstimator.getDistanceInches();
        double tx = distanceEstimator.getTx();
        double ty = distanceEstimator.getTy();

        telemetry.addData("Distance", distance);
        telemetry.addData("Tx", tx);
        telemetry.addData("Ty", ty);
        telemetry.update();
    }
}
