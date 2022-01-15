    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp(name="Savox", group="Linear Opmode")
    public class SavoxTester extends LinearOpMode {
        private DcMotor spool;

        public void runOpMode() throws InterruptedException {

            spool = hardwareMap.get(DcMotor.class, "spool");

            waitForStart();

            while (opModeIsActive()) {

                if(gamepad1.a){
                    spool.setPower(1);
                }
                else{spool.setPower(0);}
                telemetry.addData("pos: ", spool.getPower());
                telemetry.update();
            }
        }
    }
