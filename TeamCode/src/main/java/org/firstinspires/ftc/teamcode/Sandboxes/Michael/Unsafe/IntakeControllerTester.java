    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


    import org.sbs.bears.robotframework.controllers.IntakeController;
    import org.sbs.bears.robotframework.enums.IntakeSide;
    import org.sbs.bears.robotframework.enums.IntakeState;

    @TeleOp(name="Intake Controller Tester", group="Linear Opmode")
    public class IntakeControllerTester extends LinearOpMode {
        private IntakeController frontIntake;
        private boolean check = false;
        private double position = .2;

        public void runOpMode() throws InterruptedException {
            frontIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.BLUE);
            frontIntake.setState(IntakeState.BASE);
            waitForStart();

            while(opModeIsActive()){
                if(check){
                    frontIntake.checkIntake();
                }
                if(gamepad1.left_bumper){
                    check = true;
                }
                else{check = false;}
                if(gamepad1.x){
                    frontIntake.changeStatePosiiton(IntakeState.BASE, position);
                }
                if(gamepad1.b){
                    frontIntake.setState(IntakeState.DUMP);
                }
                else if(gamepad1.a){
                    frontIntake.setState(IntakeState.BASE);}
                else if(gamepad1.y){
                    frontIntake.setState(IntakeState.PARK);
                }
                else if(gamepad1.dpad_up){
                    position += .01;
                }
                else if(gamepad1.dpad_down){
                    position -= .01;
                }




                telemetry.addData("red State: ", frontIntake.getState());
                telemetry.addData("Desired Position: ", position);

                telemetry.update();
            }


        }
    }
