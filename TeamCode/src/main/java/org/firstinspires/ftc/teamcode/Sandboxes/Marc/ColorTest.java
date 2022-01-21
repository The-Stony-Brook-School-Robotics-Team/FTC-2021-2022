package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ColorTest extends OpMode {

   RevBlinkinLedDriver colorstrip2;
    @Override
    public void init() {
        colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"colorstrip");
    }

    @Override
    public void loop() {
        if(gamepad1.x) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            //colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.);

        }
        if(gamepad1.dpad_down) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        if(gamepad1.dpad_right) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
        }
        if(gamepad1.a) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); // sphere
        }
        if(gamepad1.y) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        if(gamepad1.b) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

    }
}
