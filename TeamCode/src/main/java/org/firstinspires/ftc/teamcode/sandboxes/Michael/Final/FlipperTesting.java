package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.sbs.bears.Tank.SlideConstants;


//@TeleOp(name="B - ALL SERVOS Tank2", group="Linear Opmode")
public class FlipperTesting extends LinearOpMode {


    Servo flip;
    Servo flip2;


    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;
    private int up = 1;
    private double down = SlideConstants.flipperOffset;






    public void runOpMode() throws InterruptedException {

        flip = hardwareMap.get(Servo.class, "fl");
        flip2 = hardwareMap.get(Servo.class, "fl2");

        flip.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {


            if (gamepad1.dpad_up && !qUp) {
                qUp = true;
                flip.setPosition(flip.getPosition() + .01);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);


            }
            if (!gamepad1.dpad_up && qUp) {
                qUp = false;
            }

            if (gamepad1.dpad_down && !qDown && flip.getPosition() != 0 + SlideConstants.flipperOffset + .01) {
                qDown = true;
                flip.setPosition(flip.getPosition() - .01);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);


            }
            if (!gamepad1.dpad_down && qDown) {
                qDown = false;
            }
            if (gamepad1.x) {
                flip.setPosition(up);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);

            }
            if (gamepad1.y) {
                flip.setPosition(down);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);
            }


            telemetry.addData("Current Position: ", flip.getPosition());
            telemetry.addLine();
            telemetry.addData("Hit X to set to " + up + ". Hit Y to set to " + down, "");
            telemetry.addLine();
            telemetry.update();


        }
    }
}
