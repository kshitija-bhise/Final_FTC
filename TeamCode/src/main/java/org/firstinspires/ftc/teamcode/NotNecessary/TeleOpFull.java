package org.firstinspires.ftc.teamcode.NotNecessary;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Vision.LimelightAligner;



@Disabled
@TeleOp
public class TeleOpFull extends LinearOpMode {
    LimelightAligner aligner;
    Acc acc;

    @Override
    public void runOpMode() throws InterruptedException {
        aligner = new LimelightAligner(hardwareMap);
        acc = new Acc(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.x){
                if (aligner.isAligned()){
                    aligner.stopMotors();
                    acc.setLED(0.555);
                }else {
                    aligner.alignToAprilTag();
                }
            }
            acc.setLED(0.277);

            telemetry.addData("TY", aligner.getTy());
            telemetry.update();
        }

    }
}