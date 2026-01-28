package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class LimelightAligner {

    private final DcMotor LF, LR, RF, RR;
    private final Limelight3A limelight;
    private static double Kd = 0.0000005;
    private static double kP = 0.00002;
    private final double threshold = 2.0;
    private double lasterror= 0;// degrees65

    public LimelightAligner(HardwareMap hardwareMap) {

        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);  //keep 1 for red and 0 for blue
//        telemetry.setMsTransmissionInterval(11);
        limelight.start();
    }

    public void alignToAprilTag() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            stopMotors();
            return;
        }

        double tx = result.getTx();
        double diff = -(lasterror*Kd);// LEFT / RIGHT error
        double turnPower = -((kP * tx));
        turnPower+=diff;

        if (Math.abs(tx) <= threshold) {
            stopMotors();
            return;
        }

        double minPower = 0.3;
        if (Math.abs(turnPower) < minPower) {
            turnPower = Math.signum(turnPower) * minPower;
        }

        LF.setPower(-turnPower);
        LR.setPower(-turnPower);
        RF.setPower(turnPower);
        RR.setPower(turnPower);
        lasterror=tx;
    }


    public boolean isAligned() {
        LLResult result = limelight.getLatestResult();

        if(!result.isValid()){
            return false;
        }

        return Math.abs(result.getTx()) <= threshold;
    }


    public void stopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public double getTy() {
        LLResult result = limelight.getLatestResult();

        return result.getTy();
    }

    public double getTx(){
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }
}