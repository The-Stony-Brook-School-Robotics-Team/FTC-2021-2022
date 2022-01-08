    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.sbs.bears.robotframework.controllers.IntakeController;
    import org.sbs.bears.robotframework.enums.IntakeSide;
    import org.sbs.bears.robotframework.enums.IntakeState;

    @TeleOp(name="Intake Controller Tester", group="Linear Opmode")
    public class IntakeControllerTester extends LinearOpMode {
        private IntakeController frontIntake;

        public void runOpMode() throws InterruptedException {
            frontIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.RED);
            frontIntake.setState(IntakeState.BASE);
            waitForStart();

            while(opModeIsActive()){
                frontIntake.checkIntake();
                if(gamepad1.b){
                    frontIntake.setState(IntakeState.DUMP);
                }
                if(gamepad1.a){
                    frontIntake.setState(IntakeState.BASE);}
                if(gamepad1.y){
                    frontIntake.setState(IntakeState.PARK);
                }




                telemetry.addData("red State: ", frontIntake.getState());

                telemetry.update();
            }


        }
    }
