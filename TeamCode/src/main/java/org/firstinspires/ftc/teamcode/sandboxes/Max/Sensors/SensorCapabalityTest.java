package org.firstinspires.ftc.teamcode.sandboxes.Max.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorCapabalityTest extends OpMode {
    byte[] RangeCache;
    public static final int UltrasonicSensorStartReadingValue = 0x04;
    public static final int UltrasonicSensorTotalReadingValue = 2;
    I2cDeviceSynch UltrasonicSensorEXE;
    I2cAddr UltrasonicSensorBiosAddress = new I2cAddr(0x14);
    ColorSensor Coloursensor;
    ModernRoboticsI2cRangeSensor RangeSensor;

    @Override
    public void init() {
        Coloursensor = hardwareMap.get(ColorSensor.class, "cs");
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "us");
        RangeSensor.setI2cAddress(new I2cAddr(0x14));
    }

    @Override
    public void loop() {

        telemetry.addData("COlor",Coloursensor.red());
        telemetry.addData("raw ultrasonic", RangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", RangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", RangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
