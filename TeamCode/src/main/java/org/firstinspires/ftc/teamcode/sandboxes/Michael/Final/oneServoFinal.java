package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Sensor Lift", group="Linear Opmode")

public class oneServoFinal extends LinearOpMode {


    private Servo servo = null;
    private DcMotor motor = null;
    private Rev2mDistanceSensor rev = null;
    private boolean isUp = false;



    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");
        rev = hardwareMap.get(Rev2mDistanceSensor.class, "2m");


        servo.setDirection(Servo.Direction.FORWARD);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        /**Starting in "Down"*/
        servo.setPosition(.14);
        motor.setPower(.7);



        waitForStart();


        while (opModeIsActive()) {

            if(rev.getDistance(DistanceUnit.MM) < 50 && isUp == false){
                isUp = true;
                motor.setPower(0);
                servo.setPosition(.57);

            }
            if(gamepad1.a && isUp == true){
                isUp = false;
                motor.setPower(.7);
                servo.setPosition(.14);
                servo.getPosition();

            }
            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.addData("Distance (mm)", rev.getDistance(DistanceUnit.MM));
            telemetry.addData("Is it up?", isUp);
            telemetry.update();


        }
    }
}
