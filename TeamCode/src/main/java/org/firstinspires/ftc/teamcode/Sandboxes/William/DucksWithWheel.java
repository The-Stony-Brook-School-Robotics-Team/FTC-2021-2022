package org.firstinspires.ftc.teamcode.Sandboxes.William;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Ducks with Wheel", group = "DWW")
public class DucksWithWheel extends OpMode {
    boolean pressingA = false;
    DcMotor wheelMover;
    private Double frictionConstant = 0.3;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "m1");
    }

    @Override
    public void loop() {
        wheelMover.setPower(frictionConstant);
        if (gamepad1.a && !pressingA) {
            pressingA = true;
        } else if (!gamepad1.a && pressingA) {
            frictionConstant += 0.01;
            pressingA = false;
        }

        if (gamepad1.b) {
            System.out.println(frictionConstant);
        }
    }
}
