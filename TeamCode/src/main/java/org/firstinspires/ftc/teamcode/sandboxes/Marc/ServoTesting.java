package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ServoTesting extends LinearOpMode {
    ModernRoboticsI2cRangeSensor redDevice;
    ModernRoboticsI2cRangeSensor blueDevice;
    ModernRoboticsI2cRangeSensor clawSensor;
    AnalogInput potentiomenter;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo blueExtend;
        blueExtend = hardwareMap.get(CRServo.class, "bte");
        potentiomenter = hardwareMap.get(AnalogInput.class, "po");
        redDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rd");
        blueDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        clawSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "cd");
        redDevice.setI2cAddress(I2cAddr.create8bit(0x28));
        blueDevice.setI2cAddress(I2cAddr.create8bit(0x30));
        clawSensor.setI2cAddress(I2cAddr.create8bit(0x24));
        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            if(gamepad1.a)
            {
                blueExtend.setPower(0.6);
            }
            else if(gamepad1.y)
            {
                blueExtend.setPower(-0.6);
            }
            else {
                blueExtend.setPower(0);
            }
            Log.d("po", String.valueOf(potentiomenter.getVoltage()));
            Log.d("red ad", String.valueOf(redDevice.getI2cAddress()));
            Log.d("blue ad", String.valueOf(blueDevice.getI2cAddress()));
            Log.d("red Distance", String.valueOf(redDevice.getDistance(DistanceUnit.MM)));
            Log.d("blue Distance", String.valueOf(blueDevice.getDistance(DistanceUnit.MM)));
            Log.d("claw Distance", String.valueOf(clawSensor.getDistance(DistanceUnit.MM)));

        }
    }
}
