package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Wait;

public class Acc {
    public static double shooterFarVelocity = 1550;   //1350
    public static double shooterNearVelocity = 1250;
    public double targetVelocity = 0;
    public static int targetPos = 300;
    private DcMotorEx SL;
    private DcMotorEx SW;
    private DcMotorEx SR;
    private DcMotorEx IN;
    private Servo lightServo;
    double a = 0.5;

    public Acc(HardwareMap hardwareMap) {
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        SW = hardwareMap.get(DcMotorEx.class, "SW");
        IN = hardwareMap.get(DcMotorEx.class, "IN");

        lightServo = hardwareMap.get(Servo.class,"Light");

        IN.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SL.setVelocityPIDFCoefficients(300,0,0,50);
        SR.setVelocityPIDFCoefficients(300,0,0,50);
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
        IN.setPower(0.8);
        SW.setPower(-1);
    }

    public void stopIntake() {
        IN.setPower(0);
        SW.setPower(0);
    }

    public void OutTake(){
        IN.setPower(-1);
    }

    public void setLED(double value) {
        lightServo.setPosition(value);
    }

    public boolean Goal(){
        if(Math.abs(getShooterVelocity() - targetVelocity) < 30){
            lightServo.setPosition(a);
            for(int i = 0; i < 4; i++){
                SW.setPower(-1);
                IN.setPower(0.9);
                Wait.mySleep(300);
                SW.setPower(1);
                Wait.mySleep(100);
            }
            return true;
        }
        return false;
    }

    public boolean shoot(){
        if (Math.abs(getShooterVelocity() - targetVelocity) < 30) {
            lightServo.setPosition(a);
            startIntake();
            SW.setPower(1);
            return true;
        }else{
            SW.setPower(0);
            lightServo.setPosition(0.27);
            return false;
        }

    }

    public void stopShooter(){
        SL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setPower(0);
        SR.setPower(0);
    }


    public double getShooterVelocity() {
        return SL.getVelocity();
    }

}
