package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Ducks", group = "Ducks")
public class Ducks extends LinearOpMode {
DcMotor m1;
double Gain = 0;
Boolean PressB = false;
Boolean PressY = false;
    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(true){

            telemetry.addLine()
                    .addData("Gain",Gain)
                    .addData("M1 Power: ", m1.getPower());
            telemetry.update();
            if(gamepad1.a){
            m1.setPower(Gain);
                telemetry.update();
            }else{
                m1.setPower(0);
            }

            if(gamepad1.b && !PressB){

                PressB = true;

            }else if(!gamepad1.b && PressB){
                Gain += 0.05;
                PressB = false;
            }

            if(gamepad1.y && !PressY
            ){

                PressY = true;
            }else if(!gamepad1.y && PressY){
                Gain -= 0.05;
                PressY = false;
            }

        }


    }
}
