package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PoseMemory {

    public Follower follower;
    private static Double lastX = null;
    private static Double lastY = null;
    private static Double lastHeading = null;

    // Save pose values
    public static void savePose(Pose pose) {
        if (pose == null) return;

        lastX = pose.getX();
        lastY = pose.getY();
        lastHeading = pose.getHeading();
    }

    // Print last saved values
    public static void printLastPose(Telemetry telemetry) {
        if (lastX == null) {
            telemetry.addLine("No pose saved yet.");
        } else {
            telemetry.addData("Last X", lastX);
            telemetry.addData("Last Y", lastY);
            telemetry.addData("Last Heading", lastHeading);
        }
        telemetry.update();
    }

    public static Pose getLastPose() {
        if (lastX == null) return null;
        return new Pose(lastX, lastY, lastHeading);
    }

    public static double getLastY(){
        return lastY;
    }

    public static double getLastX(){
        return lastX;
    }

    public static double getLastHeading(){
        return lastHeading;
    }

    public static boolean hasPose() {
        return lastX != null;
    }
}
