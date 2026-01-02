package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Acc;

@Configurable
public class CameraAlign {
    private Limelight3A limelight;
    private DcMotorEx RF, LR, RR, LF;
    Acc acc;

    private double previousTx = 0;
    private double previousTime = 0;
    public static double kP = 0.004;
    public static double kD = 0.0004;
    public static double maxPower = 0.45;
    public static  double dist_tolerance = 2;

    public CameraAlign(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Acc acc = new Acc(hardwareMap);

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    //  Align method with forward, strafe, and turn
    public double alignTurnValue(double manualTurn) {

        LLResult result = limelight.getLatestResult();
        double turnPower = manualTurn;

        double currentTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = currentTime - previousTime;

        if (result.isValid() && deltaTime > 0) {

            double tx = result.getTy();   // +right, -left (Limelight)
            double derivative = (tx - previousTx) / deltaTime;

            //  INVERT tx to match robot turn direction
            double autoTurn = (kP * tx) + (kD * derivative);

            if (Math.abs(tx) < dist_tolerance) {
                autoTurn = 0;
            }

            autoTurn = Math.max(-maxPower, Math.min(autoTurn, maxPower));
            turnPower = autoTurn;

            previousTx = tx;
            previousTime = currentTime;
        } else {
            previousTx = 0;
            previousTime = currentTime;
        }
        return -turnPower;
    }

    public void autoAlign(){

            LLResult result = limelight.getLatestResult();

            double currentTime = System.currentTimeMillis() / 1000.0;
            double deltaTime = currentTime - previousTime;

            if (result.isValid() && deltaTime > 0) {

                double tx = result.getTy();   // +right, -left
                double derivative = (tx - previousTx) / deltaTime;

                double autoTurn = (kP * tx) + (kD * derivative);

                // deadband
                if (Math.abs(tx) < dist_tolerance) {
                    autoTurn = 0;

                }

                autoTurn = Math.max(-maxPower, Math.min(autoTurn, maxPower));

                previousTx = tx;
                previousTime = currentTime;
            } else {
                previousTx = 0;
                previousTime = currentTime;
            }

    }

    public void autoTurn() {

        LLResult result = limelight.getLatestResult();

        double currentTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = currentTime - previousTime;

        if (!result.isValid() || deltaTime <= 0) {
            previousTime = currentTime;
            previousTx = 0;
            return;
        }

        double tx = result.getTy();   // ✅ turning uses tx
        double derivative = (tx - previousTx) / deltaTime;

        double autoTurn = (kP * tx) + (kD * derivative);

        if (Math.abs(tx) < dist_tolerance) {
            autoTurn = 0;
            acc.setLED(0.338);
        }

        follower.setTeleOpDrive(0, 0 , autoTurn, false);

//        autoTurn = Math.max(-maxPower, Math.min(autoTurn, maxPower));

        previousTx = tx;
        previousTime = currentTime;

    }

    public boolean isAligned() {

        LLResult result = limelight.getLatestResult();

        if (!result.isValid()) return false;

        return Math.abs(result.getTy()) < dist_tolerance;
    }


    private void drive(double forward, double strafe, double turn) {
        final double DEAD = 0.05;

        forward = (Math.abs(forward) < DEAD) ? 0 : forward;
        strafe = (Math.abs(strafe) < DEAD) ? 0 : strafe;
        turn = (Math.abs(turn) < DEAD) ? 0 : turn;

        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        LF.setPower(flPower);
        RF.setPower(frPower);
        LR.setPower(blPower);
        RR.setPower(brPower);
    }

    public void StopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }
}

