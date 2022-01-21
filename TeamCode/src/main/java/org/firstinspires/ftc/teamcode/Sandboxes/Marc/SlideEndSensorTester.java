package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class SlideEndSensorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel magswitch = hardwareMap.get(DigitalChannel.class, "stop");
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Magstate",!magswitch.getState());
            telemetry.update();
        }
    }
}
