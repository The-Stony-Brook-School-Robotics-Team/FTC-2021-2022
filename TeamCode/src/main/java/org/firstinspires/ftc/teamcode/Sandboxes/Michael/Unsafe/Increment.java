package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="increment", group="Linear Opmode")
public class Increment extends LinearOpMode {
    //private IntakeController frontIntake;


    public void runOpMode() throws InterruptedException {
        Servo scooper = hardwareMap.get(Servo.class, "vt");
        //DcMotor compliantWheel = hardwareMap.get(DcMotor.class, "motor");
        //Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "2m");

        scooper.setDirection(Servo.Direction.FORWARD);
        //compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        double pos = .2;
        boolean pressingB = false;
        boolean pressingA = false;
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up && !pressingA){
                pressingA = true;}
            if(!gamepad1.dpad_up && pressingA){
                pos+=.02;
                scooper.setPosition(pos);
                pressingA = false;
            }
            if(gamepad1.dpad_down && !pressingB){
                pressingB = true;}
            if(!gamepad1.dpad_down && pressingB){
                pos-=.02;
                scooper.setPosition(pos);
                pressingB = false;
            }
            if(gamepad1.a){
                scooper.setPosition(0);
            }
            if(gamepad1.b){
                scooper.setPosition(1);
            }
//BASE .4 DUMP .87  PARK .787

            telemetry.addData("Posiiton: ", scooper.getPosition());

            telemetry.update();
        }


    }
}
