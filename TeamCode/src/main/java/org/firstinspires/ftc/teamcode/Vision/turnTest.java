package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
@TeleOp
public class turnTest extends LinearOpMode {

    public Limelight3A limelight;
    DistanceEstimator distanceEstimator;
    private DcMotorEx RF, RR, LF, LR;
    private final double target_distance = 40;
    private final double dist_tolerance = 1.0;
    private final double kP = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LR = hardwareMap.get(DcMotorEx.class, "LR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        distanceEstimator = new DistanceEstimator(limelight,11,
                12.75,
                29.0);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(!distanceEstimator.hasTarget()){
                telemetry.addData("Target", "None");
                stopMotors();
                telemetry.update();
                continue;
            }

            double tx = distanceEstimator.getTy();
            double distance = distanceEstimator.getDistanceInches();

            double turnPower = (kP * tx);

            turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));


            if (tx < dist_tolerance) {
                stopMotors();
                telemetry.addLine("Aligned!");
            } else {
                turn(turnPower);
            }

            telemetry.addData("Tx", tx);
            telemetry.addData("distance", distance);
            telemetry.addData("TurnPower", turnPower);
            telemetry.addData("Tag is on", tx > 0 ? "RIGHT" : "LEFT");
            telemetry.update();
        }
    }

    public void turn(double power) {
        LF.setPower(power);
        LR.setPower(power);
        RF.setPower(-power);
        RR.setPower(-power);
    }

    public void stopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }
}
