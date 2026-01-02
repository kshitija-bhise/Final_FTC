package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class BuildPath {
    public static Path getPath(Pose startPose, Pose endPose) {
        Path newPath = new Path(new BezierLine(startPose, endPose));
        newPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        return newPath;
    }

    public static Path getPath(Pose startPose, Pose controlPoint, Pose endPose) {
        Path newPath = new Path(new BezierCurve(startPose, controlPoint, endPose));
        newPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        return newPath;
    }

    public static Path getTurnPath(Pose currentPose, double turnDegrees, boolean isLeft) {
        Path newPath = new Path(new BezierCurve(currentPose, currentPose));
        newPath.setConstantHeadingInterpolation(
                Math.toRadians(
                        normalizeAngle(convertTo360(Math.toDegrees(currentPose.getHeading())) + (isLeft ? -turnDegrees : turnDegrees))
                )
        );
        return newPath;
    }

    private static double normalizeAngle(double angle) {
        angle = angle % 360;      // Keep within -360 to 360
        if (angle < 0) {
            angle += 360;         // Convert negative angles to positive
        }
        return angle;
    }


    private static double convertTo360(double angle) {
        angle = (angle + 360) % 360;
        return angle;
    }

}
