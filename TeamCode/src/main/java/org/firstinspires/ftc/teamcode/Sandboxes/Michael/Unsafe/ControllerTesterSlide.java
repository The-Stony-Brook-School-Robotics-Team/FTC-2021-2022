    package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.sbs.bears.robotframework.controllers.*;
    import org.sbs.bears.robotframework.enums.IntakeSide;
    import org.sbs.bears.robotframework.enums.IntakeState;
    import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.SlideController;
    import org.sbs.bears.robotframework.enums.SlideState;

    @TeleOp(name="Slide Controller Tester", group="Linear Opmode")
    public class ControllerTesterSlide extends LinearOpMode {
        private SlideController slide;

        public void runOpMode() throws InterruptedException {
            slide = new SlideController(hardwareMap, telemetry);
            slide.setState(SlideState.BOTTOM);
            waitForStart();

            while(opModeIsActive()){
                if(gamepad1.a){
                    slide.setState(SlideState.TOP);}
                if(gamepad1.b){
                    slide.setState(SlideState.BOTTOM);
                }
                if(gamepad1.x){slide.setState(SlideState.MIDDLE);}


                telemetry.addData("State: ", slide.getState());

                telemetry.update();
            }


        }
    }
