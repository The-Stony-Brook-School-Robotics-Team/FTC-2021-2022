    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.sbs.bears.robotframework.controllers.IntakeController;
    import org.sbs.bears.robotframework.enums.IntakeSide;
    import org.sbs.bears.robotframework.enums.IntakeState;
    import org.sbs.bears.robotframework.enums.SlideState;

    @TeleOp(name="Slide Controller Tester", group="Linear Opmode")
    public class SlideControllerTester extends LinearOpMode {
        private SlideController slideController;

        public void runOpMode() throws InterruptedException {
            slideController = new SlideController(hardwareMap, telemetry);
            slideController.setState(SlideState.BOTTOM);
            waitForStart();

            while(opModeIsActive()){
                if(gamepad1.a){
                    slideController.setState(SlideState.TOP);}
                if(gamepad1.b){
                    slideController.setState(SlideState.BOTTOM);
                }
                if(gamepad1.x){
                    slideController.setState(SlideState.IN);
                }


                telemetry.addData("State: ", slideController.getState());
                telemetry.addData("Degrees: ", slideController.getServoPosition() * 110);
                telemetry.addData("desired pos: ", slideController.getMotorPosition());
                telemetry.addData("desired pow: ", slideController.getMotorPower());

                telemetry.update();
            }


        }
    }
