package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MaxServoStudy", group="drive")
@Disabled
public class MaxServoStudy extends LinearOpMode {

    // Configuration
    MaxServoStudyConfig configuration = new MaxServoStudyConfig();
    // Servos
    Servo Servo1;
    Servo Servo2;

    // Button Logic
    int pressingA = 0;
    int pressingB = 0;
    int pressingY = 0;
    int pressingX = 0;



    @Override
    public void runOpMode() throws InterruptedException {


        // Init
        Servo1 = hardwareMap.servo.get("servo");
        Servo2 = hardwareMap.servo.get("servo2");
        double InitialServo1Position = Servo1.getPosition();
        double DesiredServo1Position = InitialServo1Position+0.15;
        double InitialServo2Position = Servo2.getPosition();
        double DesiredServo2Position = InitialServo2Position+0.15;
        telemetry.addData("Servo One Initial Position", InitialServo1Position*360);
        telemetry.addData("Servo One Initial Position", InitialServo1Position);
        telemetry.addData("Servo Two Initial Position", InitialServo2Position*360);
        telemetry.addData("Servo Two Initial Position", InitialServo2Position);
        telemetry.update();
        // Wait For Start
        waitForStart();

        // Code before loop
        /*
        Servo1.setDirection(configuration.servoOneDirection);
        Servo2.setDirection(configuration.servoTwoDirection);
        Servo1.setPosition(0);
        Servo2.setPosition(0);
        Thread.sleep(configuration.servoWaitToReturnTime);
        Servo1.setPosition(configuration.servoOneMovementDistance);
        Servo2.setPosition(configuration.servoTwoMovementDistance);
        telemetry.addData("Servo One Position", Servo1.getPosition()*360);
        telemetry.addData("Servo Two Position", Servo2.getPosition()*360);
        telemetry.update();
        Thread.sleep(configuration.servoWaitToReturnTime);
        telemetry.addData("Servo One Position", Servo1.getPosition()*360);
        telemetry.addData("Servo Two Position", Servo2.getPosition()*360);
        Servo1.setPosition(0);
        Servo2.setPosition(0);
        telemetry.update();
        // Code while in loop
*/

        while(true){

            if(gamepad1.a && pressingA != 1) {
                pressingA = 1;
            } else if (!gamepad1.a && pressingA == 1) {
                InitialServo1Position = Servo1.getPosition();
                DesiredServo1Position = InitialServo1Position+0.15;
                InitialServo2Position = Servo2.getPosition();
                DesiredServo2Position = InitialServo2Position+0.15;
                Servo1.setPosition(configuration.servoOneMovementDistance);
                //Servo1.setPosition(DesiredServo1Position);
                Servo2.setPosition(configuration.servoTwoMovementDistance);
                //Servo2.setPosition(DesiredServo2Position);
                telemetry.addData("Servo One Position", Servo1.getPosition()*360);
                telemetry.addData("Servo Two Position", Servo2.getPosition()*360);
                telemetry.update();
                Thread.sleep(configuration.servoWaitToReturnTime);
                telemetry.addData("Servo One Position", Servo1.getPosition()*360);
                telemetry.addData("Servo Two Position", Servo2.getPosition()*360);
                //Servo1.setPosition(0);
                //Servo2.setPosition(0);
                telemetry.update();

            }
            if(gamepad1.b && pressingB != 1) {
                pressingB = 1;
            } else if (!gamepad1.b && pressingB == 1) {

                Servo1.setDirection(Servo.Direction.FORWARD);
                Servo2.setDirection(Servo.Direction.REVERSE);
                pressingB = 0;
            }
            if(gamepad1.y && pressingY != 1) {
                pressingY = 1;
            } else if (!gamepad1.y && pressingY == 1) {

                Servo1.setDirection(Servo.Direction.REVERSE);
                Servo2.setDirection(Servo.Direction.FORWARD);
                pressingY = 0;
            }
            if(gamepad1.x && pressingX != 1) {
                pressingX = 1;
            } else if (!gamepad1.x && pressingX == 1) {
            /*    Servo1.setPosition(InitialServo1Position);
                Servo2.setPosition(InitialServo2Position);
                Thread.sleep(500);*/
                Servo1.resetDeviceConfigurationForOpMode();
                Servo2.resetDeviceConfigurationForOpMode();
                Thread.sleep(configuration.servoWaitToReturnTime);
                telemetry.addData("Servo One Position", Servo1.getPosition()*360);
                telemetry.addData("Servo Two Position", Servo2.getPosition()*360);
                telemetry.update();
                pressingX = 0;
            }
            telemetry.addData("Servo One Position", Servo1.getPosition()*360);

            telemetry.addData("Servo Two Position", Servo2.getPosition()*360);

            /*

            telemetry.addData("Servo One Distance", configuration.servoOneMovementDistance);
            telemetry.addData("Servo Two Distance", configuration.servoTwoMovementDistance);
            */
            telemetry.update();

        }

    }

}
