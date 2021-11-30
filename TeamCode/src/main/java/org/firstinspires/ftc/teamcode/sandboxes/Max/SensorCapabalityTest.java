package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorCapabalityTest extends OpMode {

ColorSensor sensor;
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(ColorSensor.class, "cs");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "us");


    }

    @Override
    public void loop() {
        telemetry.addData("COlor",sensor.red());
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
