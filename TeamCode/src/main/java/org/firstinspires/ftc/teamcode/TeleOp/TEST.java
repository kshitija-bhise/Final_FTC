package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Acc;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Disabled
@TeleOp
public class TEST extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean isRobotCentric = true;
    Acc acc;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        acc = new Acc(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        if(gamepad1.right_bumper){
            slowModeMultiplier = 0.5;
        }else{
            slowModeMultiplier = 1;
        }

        if(gamepad2.right_trigger > 0.5){
            acc.startNearShoot();
            acc.shoot();
        }else if(gamepad2.left_trigger > 0.5){
            acc.startFarShoot();
            acc.shoot();
        }else{
            acc.stopShooter();
        }

        if(gamepad2.left_bumper){
            acc.startIntake();
        } else if (gamepad2.right_bumper) {
            acc.OutTake();
        } else{
            acc.stopIntake();
        }


//        if(!(gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5 && gamepad2.left_bumper)){
//            acc.stopIntake();
//        }

        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                false
        );

    }
}
