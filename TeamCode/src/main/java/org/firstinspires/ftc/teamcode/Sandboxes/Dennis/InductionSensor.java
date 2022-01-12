package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.teamcode.Sandboxes.Max.Sensors.MagneticSensor;

public class InductionSensor extends LinearOpMode {

    private static I2cDevice analogSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        analogSensor = hardwareMap.get(I2cDevice.class, "induction");
        analogSensor.readI2cCacheFromController();
    }



}
