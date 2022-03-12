package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

//@Autonomous(name = "A_William ColorSensorTest")
public class ColorSensorTest extends OpMode {
    RevColorSensorV3 colorSensorV3;


    @Override
    public void init() {
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop() {
        NormalizedRGBA color = colorSensorV3.getNormalizedColors();
        telemetry.addData("Red", color.red);
        telemetry.addData("Blue", color.blue);
        telemetry.addData("Green", color.green);
        telemetry.addData("Alpha", color.alpha);
        telemetry.update();
    }
}
