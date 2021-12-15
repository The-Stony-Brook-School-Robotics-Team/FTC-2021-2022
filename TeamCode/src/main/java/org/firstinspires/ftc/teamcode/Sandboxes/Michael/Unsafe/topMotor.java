package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


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
