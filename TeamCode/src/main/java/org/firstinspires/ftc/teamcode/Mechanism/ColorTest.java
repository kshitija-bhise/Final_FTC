package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorTest {
    NormalizedColorSensor ColorSensor;

    public enum DetectedColor{
        RED,
        GREEN,
        BLUE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap){
        ColorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        ColorSensor.setGain(4);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = ColorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Blue", normBlue);
        telemetry.addData("Green", normGreen );

        /* values of the colors

        // Green:
        R: 0.0113
        G: 0.0455
        B: 0.0526

        // Purple:
        R: 0.0275
        G: 0.0237
        B: 0.0575

        */
        return DetectedColor.UNKNOWN;

    }


}
