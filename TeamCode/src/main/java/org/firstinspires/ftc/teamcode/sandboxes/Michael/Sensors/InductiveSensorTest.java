package org.firstinspires.ftc.teamcode.sandboxes.Michael.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sandboxes.Max.Sensors.MagneticSensor;
@Disabled
@TeleOp(name = "magnetic test")
public class InductiveSensorTest extends OpMode {
    private I2cDeviceSynch magneticSwitch;
    @Override
    public void init() {
        magneticSwitch = hardwareMap.get(I2cDeviceSynch.class, "magnetic");

    }

    @Override
    public void loop() {
        telemetry.addData("addr high", magneticSwitch.read8(0x2B));
        telemetry.addData("addr high", magneticSwitch.read8(0x2A));
        telemetry.addData("addr high", magneticSwitch.getHeartbeatInterval());

        telemetry.update();
    }
}

