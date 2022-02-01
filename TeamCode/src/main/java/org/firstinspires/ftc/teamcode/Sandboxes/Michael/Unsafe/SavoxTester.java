    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp(name="Servo lolk", group="Linear Opmode")
    public class SavoxTester extends LinearOpMode {
        private Servo slideLifter;
        private boolean qA = false;
        private boolean qB = false;

        public void runOpMode() throws InterruptedException {

            slideLifter = hardwareMap.get(Servo.class, "vt");

            waitForStart();

            while (opModeIsActive()) {

                if(gamepad1.a && !qA){
                    slideLifter.setPosition(slideLifter.getPosition() + .05);
                    qA = true;
                }
                if(!gamepad1.a && qA){
                    qA = false;
                }
                if(gamepad1.b && !qB){
                    slideLifter.setPosition(slideLifter.getPosition() - .05);
                    qB = true;
                }
                if(!gamepad1.a && qB){
                    qB = false;
                }
                if(gamepad1.x)  slideLifter.setPosition(.8);
                if(gamepad1.y)  slideLifter.setPosition(.2);


                telemetry.addData("pos: ", slideLifter.getPosition());
                telemetry.update();
            }
        }
    }
