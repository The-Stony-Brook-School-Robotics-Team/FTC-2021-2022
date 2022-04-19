package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="slide measure tank")
public class SlidePositionMeasurer extends OpMode {
    private DcMotor slideMotor;
    private DcMotorEx liftMotor;
    private AnalogInput potentiometer;
    //MAX = .46;
    //AUTON = 2.7;
    //FAR = 1.1;
    @Override
    public void init() {
        slideMotor = hardwareMap.get(DcMotor.class, "spool");
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        potentiometer = hardwareMap.get(AnalogInput.class, "po");

    }
    @Override
    public void start(){
        slideMotor.setPower(.3);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(0);
    }
    @Override
    public void loop() {
   /**     if(gamepad1.dpad_up){
            slideMotor.setTargetPosition(slideMotor.getCurrentPosition()+20);
        }
        if(gamepad1.dpad_down){
            slideMotor.setTargetPosition(slideMotor.getCurrentPosition()-20);
        } **/



        telemetry.addData("slide motor", slideMotor.getCurrentPosition());
        telemetry.addData("lift motor", liftMotor.getCurrentPosition());
        telemetry.addData("po", potentiometer.getVoltage());
        telemetry.update();
    }

}
