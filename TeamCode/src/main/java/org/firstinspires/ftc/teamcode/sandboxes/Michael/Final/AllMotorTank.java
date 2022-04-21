package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name="B - ALL MOTORS Tank", group="Linear Opmode")
public class AllMotorTank extends LinearOpMode {


    DcMotor slideMotor;
    DcMotor liftMotor;
    DcMotor redIntake;
    DcMotor blueIntake;



    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;
    private int up = 1;
    private int down = 0;

    private DcMotor place;





    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotor.class, "spool");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        redIntake = hardwareMap.get(DcMotor.class, "rim");
        blueIntake = hardwareMap.get(DcMotor.class, "bim");


        DcMotor[] motors = {slideMotor, liftMotor, redIntake, blueIntake};
        String[] servoNames = {"slide extension motor", "slide lift motor", "red intake motor", "blue intake motor"};
        place = motors[0];
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !qA){
                qA = true;
                count++;
                if(count > motors.length - 1) count = 0;
                place = motors[count];
                place.setPower(place.getPower());
            }
            if(!gamepad1.a && qA){
                qA = false;
            }


            if(gamepad1.dpad_up && !qUp){
                qUp = true;
                place.setPower(place.getPower() + .01);
            }
            if(!gamepad1.dpad_up && qUp){
                qUp = false;
            }

            if(gamepad1.dpad_down && !qDown){
                qDown = true;
                place.setPower(place.getPower() - .01);
            }
            if(!gamepad1.dpad_down && qDown){
                qDown = false;
            }
            if(gamepad1.x){
                place.setPower(up);
            }
            if(gamepad1.y){
                place.setPower(down);
            }


            telemetry.addData("Current Servo: ", servoNames[count]);
            telemetry.addData("Current Position: ", place.getPower());
            telemetry.addLine();
            telemetry.addData("Hit X to set to " + up + ". Hit Y to set to " + down, "");
            telemetry.update();



        }
    }
}
