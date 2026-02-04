package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Wait;

import java.util.concurrent.locks.Lock;


@Configurable
public class Acc {
    public static double shooterFarVelocity = 1470;   //1350
    public static double shooterFarVelocityAuto = 1440;   //1350
    public static double shooterNearVelocity = 1260;
    public static double shooterNearVelocityAuto = 1200;
    public double targetVelocity = 0;
    public static int targetPos = 300;
    private DcMotorEx SL;
    private DcMotorEx SW;
    private DcMotorEx SR;
    private DcMotorEx IN;
    private Servo lightServo;

    private Servo lock;

    double a = 0.5;
    ElapsedTime parallelShoot;

    public static double kp = 500;
    public static double ki = 0;
    public static double kd = 70;
    public static double kf = 11.9;
    TelemetryManager telemetryM;


    public Acc(HardwareMap hardwareMap) {
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        SW = hardwareMap.get(DcMotorEx.class, "SW");
        IN = hardwareMap.get(DcMotorEx.class, "IN");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        lightServo = hardwareMap.get(Servo.class, "Light");
        lock = hardwareMap.get(Servo.class,"lock");

        IN.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setVelocityPIDFCoefficients(kp, ki, kd, kf);
        SR.setVelocityPIDFCoefficients(kp, ki, kd, kf);
    }


    public void startNearShootAuto(){
        SL.setVelocity(shooterNearVelocityAuto);
        SR.setVelocity(shooterNearVelocityAuto);
        targetVelocity = shooterNearVelocityAuto;
    }

    public void startFarShootAuto(){
        SL.setVelocity(shooterFarVelocityAuto);
        SR.setVelocity(shooterFarVelocityAuto);
        targetVelocity = shooterFarVelocityAuto;
    }


    public void startFarShoot() {
        SL.setVelocity(shooterFarVelocity);
        SR.setVelocity(shooterFarVelocity);
        targetVelocity = shooterFarVelocity;
        a = 0.722;
    }

    public void startNearShoot() {
        SL.setVelocity(shooterNearVelocity);
        SR.setVelocity(shooterNearVelocity);
        targetVelocity = shooterNearVelocity;
        a = 0.5;
    }

    public void startIntake() {
        IN.setPower(1);
        SW.setPower(-0.7);
    }

    public void stopIntake() {
        IN.setPower(0);
        SW.setPower(0);
    }

    public void slowIntake() {
        IN.setPower(0.4);
    }

    public void revfar() {
        SL.setVelocity(1400);
        SR.setVelocity(1400);
        targetVelocity = 1400;

    }

    public void rev() {
        SL.setVelocity(1260);
        SR.setVelocity(1260);
        targetVelocity = 1260;
    }

    public void AutoRev() {
        SL.setVelocity(shooterNearVelocityAuto);
        SR.setVelocity(shooterNearVelocityAuto);
    }

    public void AutoRevFar() {
        SL.setVelocity(shooterFarVelocityAuto);
        SR.setVelocity(shooterFarVelocityAuto);
    }



    public void OutTake() {
        IN.setPower(-0.6);
    }

    public void setLED(double value) {
        lightServo.setPosition(value);
    }

    public boolean Goal() {
        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
            lightServo.setPosition(a);
            for (int i = 0; i < 4; i++) {
                SW.setPower(1);
                Wait.mySleep(150);
                SW.setPower(-1);
                IN.setPower(0.9);
                Wait.mySleep(250);
            }
            return true;
        }
        return false;
    }

    public void ShootThree() {
        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
            for (int i = 0; i < 4; i++) {
                SW.setPower(1);
                Wait.mySleep(150);
                SW.setPower(-1);
                IN.setPower(0.9);
                Wait.mySleep(250);
            }
        }
    }


    public void ContinousShoot() {
        telemetryM.addData("Velocity", getShooterVelocity());
        telemetryM.update();
        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
            lightServo.setPosition(a);
            for (int i = 0; i < 4; i++) {
                SW.setPower(1);
                IN.setPower(0.9);
                Wait.mySleep(450);
            }
        }
    }

    public void AutoContinousShoot() {
        telemetryM.addData("Velocity", getShooterVelocity());
        telemetryM.update();
        if (Math.abs(getShooterVelocity()) > targetVelocity) {
            lightServo.setPosition(a);
                SW.setPower(1);
                IN.setPower(0.9);
                Wait.mySleep(550);
            }
        }


    public void ACS() {
        telemetryM.addData("Velocity", getShooterVelocity());
        telemetryM.update();
        if (Math.abs(getShooterVelocity()) > targetVelocity) {
            lightServo.setPosition(a);
            SW.setPower(1);
            IN.setPower(0.9);
            Wait.mySleep(450);
        }
        else{
            while(Math.abs(getShooterVelocity()) < targetVelocity) {
                Wait.mySleep(20);
                ACS();
            }
        }
    }

    public void TriggerShoot(){
        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
            IN.setPower(0.85);
            SW.setPower(1);
        }
    }

    public void initLock(){
       lock.setPosition(0);
    }

    public void releaseLock(){
        lock.setPosition(0.3);
    }


//    public void ContinousShootFar(){
//        telemetryM.addData("Velocity", getShooterVelocity());
//        telemetryM.update();
//        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
//            lightServo.setPosition(a);
//            for (int i = 0; i < 4; i++) {
//                SW.setPower(1);
//                IN.setPower(0.85);
//                Wait.mySleep(600);
//            }
//        }
//    }


    public boolean shootThree() {
        telemetryM.addData("Velocity",getShooterVelocity());
        telemetryM.update();
        if (((Math.abs(SR.getVelocity()) + Math.abs((SL.getVelocity()))) / 2) - targetVelocity > 20) {
            lightServo.setPosition(a);
            int artifacts = 0;
            IN.setPower(0.9);
            while (artifacts < 3) {
                SW.setPower(-1);
                while(!VeloShoot());
                artifacts++;
            }
            return true;
        }
        return false;
    }



    public boolean VeloShoot() {
        if (((Math.abs(SR.getVelocity()) + Math.abs((SL.getVelocity())) / 2)) - targetVelocity > 20) {
            SW.setPower(1);
            IN.setPower(0.9);
            Wait.mySleep(150);
            return true;
        } else {
            IN.setVelocity(-0.9);
            Wait.mySleep(250);
            lightServo.setPosition(0.277);
        }
        return false;
    }

    public void stopShooter() {
        SL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setPower(0);
        SR.setPower(0);
    }


    public double getShooterVelocity() {
        return SL.getVelocity();
    }


}
