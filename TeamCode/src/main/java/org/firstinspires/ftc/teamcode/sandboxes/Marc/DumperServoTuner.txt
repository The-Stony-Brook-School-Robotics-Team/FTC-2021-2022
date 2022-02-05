/*
 * Copyright (c) 2022 Copyright (c) 2022.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="A - Dumper Servo Tuner")
public class DumperServoTuner extends LinearOpMode {
    private boolean qUp,qDown;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo dumper = hardwareMap.get(Servo.class,"du");
        double position = 0.5;
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            if(gamepad1.dpad_up && !qUp)
            {
                qUp = true;
                position += 0.01;
                dumper.setPosition(position);
            }
            if(!gamepad1.dpad_up && qUp)
            {
                qUp = false;
            }
            if(gamepad1.dpad_down && !qDown)
            {
                qDown = true;
                position -= 0.01;
                dumper.setPosition(position);
            }
            if(!gamepad1.dpad_down && qDown)
            {
                qDown = false;
            }
            telemetry.addData("Servo pos",dumper.getPosition());
            telemetry.update();
        }
    }
}
