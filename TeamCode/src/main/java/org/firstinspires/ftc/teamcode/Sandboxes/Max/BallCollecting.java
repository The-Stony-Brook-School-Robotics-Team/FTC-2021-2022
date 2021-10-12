package org.firstinspires.ftc.teamcode.Sandboxes.Max;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp
public class BallCollecting extends LinearOpMode {
    private MotorEx Motor;
    private MotorEx MotorEncoder;

    @Override
    public void runOpMode() throws InterruptedException{

Motor = new MotorEx(hardwareMap, "1");
MotorEncoder = new MotorEx(hardwareMap, "1");

waitForStart();
while(true){

    Motor.setVelocity(gamepad1.left_stick_y);
    telemetry.addData("Velocity: ",MotorEncoder.getVelocity());
    telemetry.update();
}

    }
}
