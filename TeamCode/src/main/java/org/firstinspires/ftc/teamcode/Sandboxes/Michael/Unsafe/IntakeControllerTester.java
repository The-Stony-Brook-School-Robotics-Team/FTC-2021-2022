    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


    import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
    import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
    import org.sbs.bears.robotframework.enums.IntakeState;

    @TeleOp(name="Intake Controller Tester", group="Linear Opmode")
    public class IntakeControllerTester extends LinearOpMode {
        private IntakeControllerBlue blueIntake;
        private IntakeControllerRed redIntake;
        private boolean check = false;
        private double position = .2;

        public void runOpMode() throws InterruptedException {
            blueIntake = new IntakeControllerBlue(hardwareMap, telemetry);
            redIntake = new IntakeControllerRed(hardwareMap, telemetry);
            blueIntake.setState(IntakeState.BASE);
            redIntake.setState(IntakeState.BASE);
            waitForStart();

            while(opModeIsActive()){
                if(check){
                    blueIntake.checkIntake();
                    redIntake.checkIntake();
                }
                if(gamepad1.left_bumper){
                    check = true;
                }
                else{check = false;}
                if(gamepad1.x){
                    blueIntake.changeStatePosiiton(IntakeState.BASE, position);
                    redIntake.changeStatePosiiton(IntakeState.BASE, position);
                }
                if(gamepad1.b){
                    blueIntake.setState(IntakeState.DUMP);
                    redIntake.setState(IntakeState.DUMP);
                }
                else if(gamepad1.a){
                    blueIntake.setState(IntakeState.BASE);
                    redIntake.setState(IntakeState.BASE);}
                else if(gamepad1.y){
                    blueIntake.setState(IntakeState.PARK);
                    redIntake.setState(IntakeState.PARK);
                }
                else if(gamepad1.dpad_up){
                    position += .01;
                }
                else if(gamepad1.dpad_down){
                    position -= .01;
                }




                telemetry.addData("blue State: ", blueIntake.getState());
                telemetry.addData("red State: ", redIntake.getState());
                telemetry.addData("Desired Position: ", position);
                telemetry.addData("servo ", blueIntake.getServoPos());

                telemetry.update();
            }


        }
    }
