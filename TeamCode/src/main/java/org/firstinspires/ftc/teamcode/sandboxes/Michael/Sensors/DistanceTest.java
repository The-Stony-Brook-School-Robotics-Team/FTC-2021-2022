package org.firstinspires.ftc.teamcode.sandboxes.Michael.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Test")
public class DistanceTest extends LinearOpMode
{

    byte[] redCache;
    byte[] blueCache;
    ModernRoboticsI2cRangeSensor redDevice;
    ModernRoboticsI2cRangeSensor blueDevice;
    ModernRoboticsI2cRangeSensor clawSensor;
    AnalogInput potentiomenter;
    I2cDeviceSynch redSynch;
    I2cDeviceSynch blueSynch;


    @Override
    public void runOpMode() throws InterruptedException {
        potentiomenter = hardwareMap.get(AnalogInput.class, "po");
        redDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rd");
        blueDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        clawSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "cd");
        redDevice.setI2cAddress(I2cAddr.create8bit(0x28));
        blueDevice.setI2cAddress(I2cAddr.create8bit(0x30));
        clawSensor.setI2cAddress(I2cAddr.create8bit(0x24));

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("po", potentiomenter.getVoltage());
            telemetry.addData("red ad", redDevice.getI2cAddress());
            telemetry.addData("blue ad", blueDevice.getI2cAddress());
            telemetry.addData("red Distance", redDevice.getDistance(DistanceUnit.MM));
            telemetry.addData("blue Distance", blueDevice.getDistance(DistanceUnit.MM));
            telemetry.addData("claw Distance", clawSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
