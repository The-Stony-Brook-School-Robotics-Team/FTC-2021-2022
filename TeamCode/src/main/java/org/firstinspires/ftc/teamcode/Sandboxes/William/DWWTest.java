package org.firstinspires.ftc.teamcode.Sandboxes.William;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DWW Timer", group = "DWW")
public class DWWTest extends OpMode {

    private boolean isPressingA = false;
    private boolean isPressingB = false;
    private boolean wheelSpinning = false;
    private int wheelSpinningStage = 1;

    DcMotor wheelMover;
    private Double frictionConstant = 0.45;
    private Double timer;
    private Double fastTimer;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "m1");
        timer = getRuntime();
        fastTimer = getRuntime();
    }

    @Override
    public void loop() {


    }

    private void PressingA(){
        if (gamepad1.a && !isPressingA) {
            isPressingA = true;
        } else if (!gamepad1.a && isPressingA) {
            if (wheelSpinningStage == 1) {
                wheelMover.setPower(frictionConstant);
                timer = getRuntime();
                wheelSpinningStage++;
            } else if (wheelSpinningStage == 2) {
                wheelMover.setPower(1);
                wheelSpinningStage++;
                fastTimer = getRuntime();
            } else if (wheelSpinningStage == 3) {
                wheelMover.setPower(0);
                wheelSpinning = false;
                wheelSpinningStage = 1;
                telemetry.addData("Slow time", "%.4f seconds", fastTimer - timer);
                telemetry.addData("Fast time", "%.4f seconds", getRuntime() - fastTimer);
                telemetry.addData("Total time", "%.4f seconds", getRuntime() - timer);
                telemetry.update();
            }
            isPressingA = false;
        }
    }
}