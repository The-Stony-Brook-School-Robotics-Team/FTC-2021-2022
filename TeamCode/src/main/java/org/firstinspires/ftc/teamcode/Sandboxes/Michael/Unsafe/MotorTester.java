package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideState;

@TeleOp(name="Slide Motor Tester", group="Linear Opmode")
public class MotorTester extends LinearOpMode {
    private DcMotor motor;
    private int ticks = 100;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "spool");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                ticks--;}
            if(gamepad1.b){
                ticks++;
            }
            if(gamepad1.x){
                motor.setTargetPosition(ticks);
            }
            if(gamepad1.y){
                motor.setPower(.5);
            }


            telemetry.addData("Ticks: ", motor.getTargetPosition());
            telemetry.addData("Power: ", motor.getPower());

            telemetry.update();
        }


    }
}
