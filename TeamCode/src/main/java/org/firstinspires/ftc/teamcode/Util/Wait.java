package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait {
    public static void mySleep(int milliseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < milliseconds);
    }
}
