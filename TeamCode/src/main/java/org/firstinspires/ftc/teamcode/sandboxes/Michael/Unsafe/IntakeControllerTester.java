    package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
    import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
    import org.sbs.bears.robotframework.controllers.SlideController;
    import org.sbs.bears.robotframework.enums.IntakeState;

    @TeleOp(name="Intake Controller Tester", group="Linear Opmode")
    public class IntakeControllerTester extends LinearOpMode {
        private IntakeControllerBlue blueIntake;
        private IntakeControllerRed redIntake;
        private SlideController slideController;
        private boolean qA = false;
        private boolean qB = false;
        private double position = .03;

        public void runOpMode() throws InterruptedException {
            slideController = new SlideController(hardwareMap, telemetry);
            blueIntake = new IntakeControllerBlue(hardwareMap, slideController.dumperServo, telemetry);
            //redIntake = new IntakeControllerRed(hardwareMap, telemetry);
            blueIntake.setState(IntakeState.DUMP);
            //redIntake.setState(IntakeState.PARK);
            waitForStart();

            while(opModeIsActive()){
                if(gamepad1.right_bumper){
                    blueIntake.checkIntake();
              //      redIntake.checkIntake();

                }

                if(gamepad1.x){
                    blueIntake.changeStatePosiiton(IntakeState.BASE, position);
                    //redIntake.changeStatePosition(IntakeState.BASE, position);
                }
                if(gamepad1.b){
                    //blueIntake.setState(IntakeState.DUMP);
                    blueIntake.setState(IntakeState.DUMP);
                 //   redIntake.setState(IntakeState.DUMP);
                }
                else if(gamepad1.a){
                    //blueIntake.setState(IntakeState.BASE);
                    blueIntake.setState(IntakeState.BASE);
                  //  redIntake.setState(IntakeState.BASE);
                }
                else if(gamepad1.y){
                    //blueIntake.setState(IntakeState.PARK);
                    blueIntake.setState(IntakeState.DUMP);
                  //  redIntake.setState(IntakeState.PARK);
                }
                else if(gamepad1.dpad_up && !qA){
                    //position += .005;
                    blueIntake.timeToOpenStopper += 50;
                    qA = true;
                }
                else if(!gamepad1.dpad_up && qA){
                    qA = false;
                }
                else if(gamepad1.dpad_down && !qB){
                    //position -= .005;
                    blueIntake.timeToOpenStopper -= 50;
                    qB = true;
                }
                else if(!gamepad1.dpad_down && qB){
                    qB = false;
                }




                telemetry.addData("blue State: ", blueIntake.getState());
               // telemetry.addData("red State: ", redIntake.getState());
                telemetry.addData("Sleep: ", blueIntake.timeToOpenStopper);
                telemetry.addData("blue servo ", blueIntake.getServoPos());
                //telemetry.addData("red servo ", redIntake.getServoPos());
               // telemetry.addData("red distance", redIntake.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("blue distance", blueIntake.distanceSensor.getDistance(DistanceUnit.MM));

                telemetry.update();
            }


        }
    }
