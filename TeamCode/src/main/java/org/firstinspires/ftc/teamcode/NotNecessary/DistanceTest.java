package org.firstinspires.ftc.teamcode.NotNecessary;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;

@Disabled
@TeleOp
public class DistanceTest extends OpMode {
    private DistanceEstimator distanceEstimator;

    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        distanceEstimator = new DistanceEstimator(limelight, 0, 15.5, 29.5);

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
