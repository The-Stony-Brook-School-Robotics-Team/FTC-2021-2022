package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.distribution.TDistribution;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsTank;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "A - Tank Driving Tester")
@Config
public class TankDrivingTester extends LinearOpMode {
    private boolean qB;
    public static int STATE = 0;
    public static double toTravel = 10;
    public static double speed = 0.2;
    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    public static double angle = 45;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        AtomicReference<Boolean> terminate_signal = new AtomicReference<>();
        terminate_signal.set(false);
        waitForStart();
        boolean qA = false;
        while(opModeIsActive() && !isStopRequested())
        {
            //driver.RR.setMotorPowers(-gamepad1.left_stick_x +gamepad1.right_stick_x, -gamepad1.left_stick_x - gamepad1.right_stick_x);
            if((gamepad1.a && !qA) || STATE == 1)
            {
                qA = true;
                driver.stopMotors();
                driver.goBackwardGyroPIDAsync(toTravel,speed,terminate_signal,P,I,D);
                STATE = 0;
            }
            if(!gamepad1.a && qA)
            {
                qA = false;
            }
            if((gamepad1.b && !qB) || STATE == 2)
            {
                qB = true;
                driver.stopMotors();
                driver.goForwardGyroPIDAsync(toTravel,speed,terminate_signal,P,I,D);
                STATE = 0;
            }
            if(!gamepad1.b && qB)
            {
                qB = false;
            }
            if(STATE == 3)
            {
                driver.shutDown();
                STATE = 0;
            }
            if(STATE == 4)
            {
                driver.turnL(angle,speed);
                STATE = 0;
            }
            if(STATE == 5)
            {
                driver.turnR(angle,speed);
                STATE = 0;
            }
            if(STATE == 6)
            {
                driver.RRturnR(angle);
                STATE = 0;
            }
            if(STATE == 7)
            {
                driver.RRturnL(angle);
                STATE = 0;
            }
        }
        driver.shutDown();
        terminate_signal.set(true);
    }
}
