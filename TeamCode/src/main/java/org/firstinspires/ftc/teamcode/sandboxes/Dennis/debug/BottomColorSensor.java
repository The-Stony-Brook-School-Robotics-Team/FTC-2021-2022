package org.firstinspires.ftc.teamcode.sandboxes.Dennis.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "D - Bottom Color Sensor Tester")
public class BottomColorSensor extends LinearOpMode {

    ColorRangeSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");

        waitForStart();

        while(!isStopRequested()) {
            telemetry.addData("color sensor: ", colorSensor.argb());
            telemetry.update();
        }


    }
}
