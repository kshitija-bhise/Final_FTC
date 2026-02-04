package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.6)
            .forwardZeroPowerAcceleration(-26.7063)
            .lateralZeroPowerAcceleration(-49.1865)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.005, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.06, 0.01))  old values
            .headingPIDFCoefficients(new PIDFCoefficients(1.25, 0, 0.1, 0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0 , 0.000009, 0.6, 0.01));
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0 , 0.000001, 0.6, 0.02));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("RF")
            .rightRearMotorName("RR")
            .leftFrontMotorName("LF")
            .leftRearMotorName("LR")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(74.9267)
            .yVelocity(59.3651);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.53)
            .strafePodX(-3.74)
//            .forwardPodY(2.5)
//            .strafePodX(-2.5)
//            .forwardPodY(-3.5)
//            .strafePodX(-2.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("PinPoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static double LATERAL_MULTIPLIER = 1.30;

}