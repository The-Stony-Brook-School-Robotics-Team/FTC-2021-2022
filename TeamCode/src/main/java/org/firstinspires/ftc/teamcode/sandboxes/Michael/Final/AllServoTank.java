package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.SlideConstants;

@Config
@TeleOp(name="B - ALL SERVOS Tank", group="Linear Opmode")
public class AllServoTank extends LinearOpMode {


    Servo blueTapeRotate; //.47
    Servo blueTapeTilt; //.49
    Servo blueTapeExtend;
    Servo blueIntakeServo;



    Servo redTapeRotate;
    Servo redTapeTilt;
    Servo redTapeExtend;
    Servo redIntakeServo;
    Servo claw;
    Servo flip;
    Servo flip2;

    DcMotorEx spool;
    AnalogInput pot;

    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;
    private int up = 1;
    private int down = 0;

    private Servo place;





    public void runOpMode() throws InterruptedException {
        blueTapeRotate = hardwareMap.get(Servo.class, "btr");
        blueTapeTilt = hardwareMap.get(Servo.class, "btt");
        blueTapeExtend = hardwareMap.get(Servo.class, "bte");
        blueIntakeServo = hardwareMap.get(Servo.class, "bi");

        redTapeRotate = hardwareMap.get(Servo.class, "rtr") ;
        redTapeTilt = hardwareMap.get(Servo.class, "rtt");
        redTapeExtend = hardwareMap.get(Servo.class, "rte");
        redIntakeServo = hardwareMap.get(Servo.class, "ri");
        claw = hardwareMap.get(Servo.class, "cl");
        flip = hardwareMap.get(Servo.class, "fl");
        flip2 = hardwareMap.get(Servo.class, "fl2");

        flip.setDirection(Servo.Direction.REVERSE);

        spool = hardwareMap.get(DcMotorEx.class, "spool");
        pot = hardwareMap.get(AnalogInput.class, "po");

        Servo[] servos = {blueTapeRotate, blueTapeTilt, blueTapeExtend, blueIntakeServo, redTapeRotate, redTapeTilt, redTapeExtend, redIntakeServo, claw, flip, flip2};
        String[] servoNames = {"blue tape rotate", "blue tape tilt ", "blue tape extend ", "blue intake", "red tape rotate", "red tape tilt", "red tape extend", "red intake", "claw", "flip", "flip2"};
        place = servos[0];
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !qA){
                qA = true;
                count++;
                if(count > servos.length - 1) count = 0;
                place = servos[count];
                place.setPosition(place.getPosition());
            }
            if(!gamepad1.a && qA){
                qA = false;
            }


            if(gamepad1.dpad_up && !qUp){
                qUp = true;
                place.setPosition(place.getPosition() + .01);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);


            }
            if(!gamepad1.dpad_up && qUp){
                qUp = false;
            }

            if(gamepad1.dpad_down && !qDown){
                qDown = true;
                place.setPosition(place.getPosition() - .01);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);


            }
            if(!gamepad1.dpad_down && qDown){
                qDown = false;
            }
            if(gamepad1.x){
                place.setPosition(up);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);

            }
            if(gamepad1.y){
                place.setPosition(down);
                flip2.setPosition(flip.getPosition() - SlideConstants.flipperOffset);
            }


            telemetry.addData("Current Servo: ", servoNames[count]);
            telemetry.addData("Current Position: ", place.getPosition());
            telemetry.addLine();
            telemetry.addData("Hit X to set to " + up + ". Hit Y to set to " + down, "");
            telemetry.addLine();
            telemetry.addData("Potentiometer: ", pot.getVoltage());
            telemetry.addData("Slide Motor: ", spool.getCurrentPosition());
            telemetry.update();



        }
    }
}
