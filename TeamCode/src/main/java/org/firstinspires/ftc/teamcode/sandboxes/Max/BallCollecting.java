package org.firstinspires.ftc.teamcode.sandboxes.Max;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//@TeleOp
public class BallCollecting extends LinearOpMode {
    private DcMotor Motor;

    private boolean pressingArrowUp = false;
    private boolean pressingArrowDown = false;
    private double voltage = 0;

    private boolean pressingA = false;

    @Override
    public void runOpMode() throws InterruptedException{
        Motor = hardwareMap.get(DcMotor.class, "m1");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_up && !pressingArrowUp) {
                pressingArrowUp = true;
            } else if(!gamepad1.dpad_up && pressingArrowUp) {
                voltage = voltage + 0.05;
                pressingArrowUp = false;
            }

            if(gamepad1.dpad_down && !pressingArrowDown) {
                pressingArrowUp = true;
            } else if(!gamepad1.dpad_down && pressingArrowDown) {
                voltage = voltage - 0.05;
                pressingArrowUp = false;
            }

            if(gamepad1.left_stick_y < 0.1) {
                voltage = gamepad1.left_stick_y;
            }

            if(gamepad1.a && !pressingA) {
                pressingA = true;
            } else if(!gamepad1.a && pressingA) {
                // Friction
                // TODO: Test this constant
                double Mews = 0.1;

                double sqrtOfR;
                double squareResult = 1 / (Math.sqrt(Mews));



                pressingA = false;
            }

            Motor.setPower(voltage);
            telemetry.addData("Power: ", Motor.getPower());
            telemetry.update();
        }

    }
}
