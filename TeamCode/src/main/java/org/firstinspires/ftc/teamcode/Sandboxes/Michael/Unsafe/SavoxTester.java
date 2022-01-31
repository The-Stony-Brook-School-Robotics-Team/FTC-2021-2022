    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp(name="Servo lolk", group="Linear Opmode")
    public class SavoxTester extends LinearOpMode {
        private Servo dumper;
        private boolean qA = false;
        private boolean qB = false;

        public void runOpMode() throws InterruptedException {

            dumper = hardwareMap.get(Servo.class, "du");

            waitForStart();

            while (opModeIsActive()) {

                if(gamepad1.a && !qA){
                    dumper.setPosition(dumper.getPosition() + .05);
                    qA = true;
                }
                if(!gamepad1.a && qA){
                    qA = false;
                }
                if(gamepad1.b && !qB){
                    dumper.setPosition(dumper.getPosition() - .05);
                    qB = true;
                }
                if(!gamepad1.a && qB){
                    qB = false;
                }

                telemetry.addData("pos: ", dumper.getPosition());
                telemetry.update();
            }
        }
    }
