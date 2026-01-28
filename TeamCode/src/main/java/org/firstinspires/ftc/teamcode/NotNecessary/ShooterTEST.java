package org.firstinspires.ftc.teamcode.NotNecessary;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;



@Disabled
@Configurable
@TeleOp
public class ShooterTEST extends OpMode {
    public DcMotorEx SL;
    public DcMotorEx SR;

    public double highVelocity = 1500;

    public double lowVelocity = 1000;

    public double targetVelocity = highVelocity;

    static double F = 0.3;

    static double P = 10.5;

    static double Kd = 0.0005;

    double [] stepSize = {10.0, 1.0 , 0.1 , 0.001, 0.0001};

    int stepIndex = 1;
    TelemetryManager telemetryManager;


    @Override
    public void init() {
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        SR.setDirection(DcMotorSimple.Direction.REVERSE);
        SL.setDirection(DcMotorSimple.Direction.FORWARD);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,Kd, F);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("INITIALIZATION COMPLETE");

    }

    @Override
    public void loop() {
        //get all your command
        //set target velocity
        //update telemetry

        if (gamepad1.yWasPressed()){
            if(targetVelocity == highVelocity){
                targetVelocity = lowVelocity;
            }else{
                targetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSize.length;
            F = stepSize[stepIndex];
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSize[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSize[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSize[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSize[stepIndex];
        }



        //set new PIDFCoefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0, F);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        SR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set target velocity
        SL.setVelocity(targetVelocity);
        SR.setVelocity(targetVelocity);

        double currentVelo = SL.getVelocity();
        double error = targetVelocity - currentVelo;

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelo);
        telemetry.addData("Error", error);
        telemetryManager.addData("Current Vel",currentVelo);
        telemetry.addLine("------------------------------");

        telemetry.addData("Tuning P, %.4f(U/D)", P);
        telemetry.addData("Tuning F, %.4f(L/R)", F);
        telemetry.update();
        telemetryManager.update();
    }

}

