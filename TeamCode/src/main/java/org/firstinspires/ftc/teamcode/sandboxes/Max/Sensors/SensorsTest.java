package org.firstinspires.ftc.teamcode.sandboxes.Max.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name="SensorTest", group = "drive")
public class SensorsTest extends OpMode {

    byte[] RangeCache;
    I2cDevice UltrasonicSensor;
    I2cDeviceSynch UltrasonicSensorEXE;
    I2cAddr UltrasonicSensorBiosAddress = new I2cAddr(0x14);
    public static final int UltrasonicSensorStartReadingValue = 0x04;
    public static final int UltrasonicSensorTotalReadingValue = 2;

    FtcDashboard FTCDashBoard;


    Sensor MagneticSensor;
    Sensor LightSensor;


    @Override
    public void init() {
        UltrasonicSensor = hardwareMap.i2cDevice.get("us");
        UltrasonicSensorEXE = new I2cDeviceSynchImpl(UltrasonicSensor, UltrasonicSensorBiosAddress, false);
        UltrasonicSensorEXE.engage();



    }

    @Override
    public void loop() {

        RangeCache = UltrasonicSensorEXE.read(UltrasonicSensorStartReadingValue,UltrasonicSensorTotalReadingValue);
        telemetry.addData("Range", RangeCache[0]&0xFF);
        telemetry.addData("ODS", RangeCache[1]&0xFF);
        telemetry.update();


    }
}
