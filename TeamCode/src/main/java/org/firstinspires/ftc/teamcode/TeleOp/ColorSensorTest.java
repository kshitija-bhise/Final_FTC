package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ColorTest;


@Disabled
@TeleOp
public class ColorSensorTest extends OpMode {

    ColorTest colorTest = new ColorTest();

    @Override
    public void init() {
        colorTest.init(hardwareMap);
    }

    @Override
    public void loop() {
        colorTest.getDetectedColor(telemetry);
    }
}
