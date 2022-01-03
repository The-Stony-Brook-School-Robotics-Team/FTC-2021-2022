package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Vex Motor Test", group="Linear Opmode")

public class VexMotorTest extends LinearOpMode {

    private CRServo vexMotor;
    private double pw = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.dpad_up){vexMotor = hardwareMap.get(CRServo.class, "vex"); vexMotor.setPower(1);}
            if(gamepad1.dpad_down){vexMotor = null;}
        }



    }
}
