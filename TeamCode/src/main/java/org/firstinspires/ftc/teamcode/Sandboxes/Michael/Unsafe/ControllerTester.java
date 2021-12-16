    package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.sbs.bears.robotframework.controllers.IntakeController;
    import org.sbs.bears.robotframework.enums.IntakeSide;
    import org.sbs.bears.robotframework.enums.IntakeState;

    @TeleOp(name="Controller Tester", group="Linear Opmode")
    public class ControllerTester extends LinearOpMode {
        private IntakeController frontIntake;
        private IntakeController backIntake;

        public void runOpMode() throws InterruptedException {
            frontIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.FRONT);
            backIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.BACK);
            frontIntake.setState(IntakeState.BASE);
            backIntake.setState(IntakeState.BASE);
            waitForStart();

            while(opModeIsActive()){

                if(gamepad1.a){
                    frontIntake.setState(IntakeState.BASE);}
                if(gamepad1.b){
                    backIntake.setState(IntakeState.BASE);}
                if(gamepad1.x){
                    frontIntake.setState(IntakeState.DUMP);}
                if(gamepad1.y){
                    backIntake.setState(IntakeState.DUMP);}


                telemetry.addData("Front State: ", frontIntake.getState());
                telemetry.addData("Back State: ", backIntake.getState());
                telemetry.update();
            }


        }
    }
