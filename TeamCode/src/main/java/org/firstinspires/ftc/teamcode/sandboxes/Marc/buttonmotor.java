package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.archive.MotorEncoderController.motorNames;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class buttonmotor extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    //public static

    @Override
    public void runOpMode() throws InterruptedException {


        for (int i = 0; i < 4; i++) {
            motors[i] = (hardwareMap.get(DcMotor.class, motorNames[i]));
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

        while(opModeIsActive()) {


            if(gamepad1.a) {
                motors[0].setPower(1); //right
                continue;
            }
            if(gamepad1.b) {
                motors[1].setPower(1); //back
                continue;
            }
            if(gamepad1.y) {
                motors[2].setPower(1);                continue; //left
            }
            if(gamepad1.x) {
                motors[3].setPower(1);                continue; //lf
            }
            stopMotors();



        }

    }
    public void forwardPow(double pow) {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(pow);
        }
    }
    public void stopMotors()
    {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
    }

}
