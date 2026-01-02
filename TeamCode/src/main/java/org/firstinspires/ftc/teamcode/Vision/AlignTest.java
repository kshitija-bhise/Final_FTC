package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;


@Disabled
@TeleOp
public class AlignTest extends LinearOpMode {
    DcMotor LF, LR, RF, RR;
    private Limelight3A limelight;
    DistanceEstimator distanceEstimator;

    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "FR");
        RR = hardwareMap.get(DcMotor.class, "RR");

        distanceEstimator = new DistanceEstimator(limelight, 17.0, 12.5, 29.0);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
           if(!distanceEstimator.hasTarget()){
               telemetry.addData("No Target","Found");
               continue;
           }

           double tx = distanceEstimator.getTx();
           double distance = distanceEstimator.getDistanceInches();

           telemetry.addData("Tx", tx);
           telemetry.addData("Distance", distance);
           telemetry.update();

           
        }
        limelight.stop();
    }

    public void drive(double x, double y, double z){

        final double DEAD = 0.05;
        x = (Math.abs(x) < DEAD) ? 0 : x;
        y = (Math.abs(y) < DEAD) ? 0 : y;
        z = (Math.abs(z) < DEAD) ? 0 : z;

        double flPower = x + y + z;
        double blPower = x - y + z;
        double frPower = x - y - z;
        double brPower = x + y - z;

        LF.setPower(flPower);
        LR.setPower(blPower);
        RF.setPower(frPower);
        RR.setPower(brPower);
    }
}