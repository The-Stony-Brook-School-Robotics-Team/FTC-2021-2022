    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp(name="Savox", group="Linear Opmode")
    public class SavoxTester extends LinearOpMode {
        private Servo savox;
        boolean pressingUp = false;
        boolean pressingDown = false;
        public void runOpMode() throws InterruptedException {

            savox = hardwareMap.get(Servo.class, "vt");

            waitForStart();

            while(opModeIsActive()){
                if(gamepad1.dpad_up && !pressingUp){
                    pressingUp = true;
                }
                else if(!gamepad1.dpad_up && pressingUp){
                    savox.setPosition(savox.getPosition()+.05);
                    pressingUp = false;
                }
                if(gamepad1.dpad_down && !pressingDown){
                    pressingDown = true;
                }
                else if(!gamepad1.dpad_down && pressingDown){
                    savox.setPosition(savox.getPosition()-.05);
                    pressingDown = false;
                }
                if(gamepad1.a){
                telemetry.addData("pos: ", savox.getPosition());
                telemetry.update();
            }


        }
    }
