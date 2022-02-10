package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="B- INTAKE TESTING", group="Linear Opmode")
public class IntakeTesting extends LinearOpMode {
    private Servo bucket;
    private Servo red;
    private Servo blue;
    private Servo place;
    private boolean qA = false;
    private boolean qB = false;

    public double dumperPosition_CLOSED = 0.33;  // remeasured on jan 31 at 16h08
    public double dumperPosition_READY = 0.53;
    public double dumperPosition_EJECT = 0.74;
    public double dumperPosition_RETRACTING = 0.05;

    public void runOpMode() throws InterruptedException {
        red = hardwareMap.get(Servo.class, "ri");
        blue = hardwareMap.get(Servo.class, "bi");
        place = red;


        waitForStart();

        while (opModeIsActive()) {


        }
    }
}
