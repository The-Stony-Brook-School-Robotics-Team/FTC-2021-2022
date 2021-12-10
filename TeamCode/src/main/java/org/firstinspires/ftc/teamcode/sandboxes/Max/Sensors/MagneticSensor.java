package org.firstinspires.ftc.teamcode.sandboxes.Max.Sensors;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class MagneticSensor extends LinearOpMode {
TouchSensor MagneticSensor;
RevTouchSensor DigChannel;
    //private DigitalInput limitSwitch = new DigitalInput(DIGITAL_INPUT_PORT);
    @Override
    public void runOpMode() throws InterruptedException {
MagneticSensor = hardwareMap.get(TouchSensor.class, "mg");
//DigChannel = hardwareMap.get(RevTouchSensor.class, "mgd");

    waitForStart();

        while (true){
/*
            telemetry.addLine()
            .addData("Magnetic", "%.3f", MagneticSensor.getValue())
            .addData("Configuration",MagneticSensor.getConnectionInfo());
            //.addData("Rev Touch", "%.3f", DigChannel.getValue());
 */
            telemetry.update();

            if (MagneticSensor.isPressed() == true) {
                telemetry.addData("MagneticSensor", "Is Pressed");
                telemetry.update();
            } else {
                telemetry.addData("MagneticSensor", "Is Not Pressed");
                telemetry.update();

//                if (DigChannel.isPressed() == true) {
//                    telemetry.addData("Digital Touch", "Is Pressed");
//                    telemetry.update();
//                } else {
//                    telemetry.addData("Digital Touch", "Is Not Pressed");
//                    telemetry.update();
//            }

        }


    }
}
}
