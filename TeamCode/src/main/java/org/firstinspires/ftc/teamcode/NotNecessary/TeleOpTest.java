package org.firstinspires.ftc.teamcode.NotNecessary;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanism.Acc;
import org.firstinspires.ftc.teamcode.Vision.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Disabled
@TeleOp
public class TeleOpTest extends LinearOpMode {
    Follower follower;
    CameraAlign cameraAlign;
    DistanceEstimator distanceEstimator;
    Limelight3A limelight;
    Acc acc;
    private Supplier<PathChain> pathChain;
    public boolean automatedDrive = false;
    public boolean shootfar;
    public boolean shootfardone = false;
    public boolean shootneardone = false;
    public PathChain pathsFar;
    public PathChain pathsNear;
    public DcMotor RF, RR, LR, LF;

    double desiredAngleDeg = 315;

    boolean isTeleOpDriveStarted = false;


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);

        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");

        LR.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setZeroPowerBehavior(BRAKE);
        RR.setZeroPowerBehavior(BRAKE);
        LR.setZeroPowerBehavior(BRAKE);
        LF.setZeroPowerBehavior(BRAKE);

        acc = new Acc(hardwareMap);

        follower.setStartingPose(new Pose(50, 25, Math.toRadians(0)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cameraAlign = new CameraAlign(hardwareMap);
        distanceEstimator = new DistanceEstimator(
                limelight,
                6.0,
                11.7,
                29.5
        );

        follower.update();

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx= gamepad1.left_stick_x;

            boolean alignment = false;
            Pose pose = follower.getPose();
            double xRobo = follower.getPose().getX();
            double yRobo = follower.getPose().getY();
            double headingDeg = Math.toDegrees(pose.getHeading());
            if (gamepad1.left_trigger > 0.1) {
                alignment = true;
            }
                if (gamepad1.left_bumper){
                    follower.breakFollowing();
                    drive(y, x, rx);
                }

            pathsFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(xRobo, yRobo), new Pose(58.000, -30.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headingDeg), Math.toRadians(315))
                    .build();
            pathsNear = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(xRobo, yRobo), new Pose(55.000, 23.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(headingDeg), Math.toRadians(293))
                    .build();


            if (alignment && !shootneardone && !shootfardone) {
                double ShootNear = Math.sqrt((((xRobo - 58) * (xRobo - 58)) + ((yRobo + 30) * (yRobo + 30))));
                double ShootFar = Math.sqrt((((xRobo - 55) * (xRobo - 55)) + ((yRobo - 23) * (yRobo - 23))));
                if (ShootNear < ShootFar) {
                    shootfar = false;
                } else {
                    shootfar = true;
                }
                if (!shootfar && !(follower.atPose(new Pose(58, -30), 2, 2, Math.toRadians(2)))) {

                    follower.followPath(pathsFar);
                    shootfardone = true;

                }
                if (shootfar && !follower.atPose(new Pose(55, 23), 2, 2, Math.toRadians(2))) {
                    follower.resumePathFollowing();
                    follower.followPath(pathsNear);
                    shootneardone = true;
                }
            }
            if (gamepad1.aWasPressed() && !shootfar && !(follower.atPose(new Pose(58, -30), 2, 2, Math.toRadians(2)))) {
                follower.followPath(pathChain.get()); //Lazy Curve Generation
                automatedDrive = true;
                shootfardone = true;
            }

            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }




            if (gamepad1.right_bumper) {
                follower.resumePathFollowing();
                telemetry.addData("Right bumper", "is pressed");
            }


            telemetry.addData("Near", shootneardone);
            telemetry.addData("Far", shootfardone);
            telemetry.addData("alignment", alignment);
            telemetry.addData("X", xRobo);
            telemetry.addData("Y", yRobo);
            telemetry.addData("Heading", headingDeg);
            telemetry.update();

        }

    }private void drive(double forward, double strafe, double turn) {
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


