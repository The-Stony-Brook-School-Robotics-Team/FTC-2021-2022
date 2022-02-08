package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MaxServoStudyConfig {

    // Servos
    public static double servoOneMovementDistance = 0.18;
    public static double servoTwoMovementDistance = 0.18;

    // Directions
    public static Servo.Direction servoOneDirection = Servo.Direction.FORWARD;
    public static Servo.Direction servoTwoDirection = Servo.Direction.REVERSE;

    // Sleep times
    public static int servoWaitToReturnTime = 1000;

}
