package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.sbs.bears.Tank.NewSlideController;
import org.sbs.bears.Tank.SlideConstants;
import org.sbs.bears.Tank.TipPreventionController;
import org.sbs.bears.robotframework.Sleep;

@TeleOp
public class SlideTesting extends LinearOpMode {
    boolean qA;
    boolean qB;
    private boolean qC;
    private AnalogInput potentiometer;

    @Override
    public void runOpMode() throws InterruptedException {
        //NewSlideController slide = new NewSlideController(hardwareMap);
        //TipPreventionController preventer = new TipPreventionController(hardwareMap);
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectInitLoop = Integer.MAX_VALUE;
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        potentiometer = hardwareMap.get(AnalogInput.class, "po");
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
          /*  if(gamepad1.a && !qA)
            {
                qA = true;
                Log.d("SlideTesting","Begin extenddropretract");
                slide.extendDropRetract(SlideConstants.slideMotorPosition_THREE_CLOSE,SlideConstants.flipper_THREE_CLOSE,SlideConstants.potentiometer_THREE_DEPOSIT);
                Log.d("SlideTesting","end extenddropretract");
            }
            else if(!gamepad1.a && qA)
            {
                qA = false;
            }
            if(gamepad1.dpad_right && !qB)
            {
                qB = true;
            }
            else if(!gamepad1.dpad_right && qB)
            {
                qB = false;
            }
            */if(gamepad1.dpad_left && !qC)
            {
                qC = true;
                double desiredVoltage = 2;
                if(potentiometer.getVoltage() < desiredVoltage) {
                    while (potentiometer.getVoltage() < desiredVoltage) {
                        liftMotor.setPower(SlideConstants.liftMotorPower_MOVING);
                    }
                    liftMotor.setPower(0);
                    return;
                }
                while (potentiometer.getVoltage() > desiredVoltage){
                    try {
                        liftMotor.setPower(-SlideConstants.liftMotorPower_MOVING);
                    }
                    catch (Exception e)
                    {
                        //e.printStackTrace();
                        Log.d("NewSlideController","Slide height failed");
                    }
                }
            }
            else if(!gamepad1.dpad_left && qC)
            {
                qC = false;
            }
            if(gamepad1.b)
            {
                Log.d("SlideTesting","Setting positive power: should go down");
                Log.d("SlideTesting","Potentiometer: " + potentiometer.getVoltage());
                liftMotor.setPower(0.2);
            }
            if(gamepad1.x)
            {
                liftMotor.setPower(-0.2);
            }
            else if (!gamepad1.x && !gamepad1.b)
            {
                //Log.d("SlideTesting","Setting zero power: should stay still");
                liftMotor.setPower(0);
            }

        }

        Log.d("SlideTesting","end opmode");
    }
}
