package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Duck Spin", group="Linear Opmode")

public class topMotor extends LinearOpMode {
  private DcMotor topMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        topMotor = hardwareMap.get(DcMotor.class, "duck");
        while (opModeIsActive()) {
            if(gamepad1.a){topMotor.setPower(1);}
            if(gamepad1.b){topMotor.setPower(-1);}
            if(gamepad1.x){topMotor.setPower(0);}
        }
    }
}
