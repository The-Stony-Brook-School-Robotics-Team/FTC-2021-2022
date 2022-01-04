package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideComponents;
import org.sbs.bears.robotframework.enums.SlideState;

import java.util.HashMap;

public class SlideController {

    private Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;

    private DcMotor slideMotor;


    /** Vertical Servo, then Horizontal Servo, then Dumper Servo, then Slide Motor, */
    private HashMap<SlideComponents, Double> values = new HashMap<>();

    private double[] bottomPos = {0.0, 0.0}; //.141
    private double[] middlePos = {.5, 0.0};
    private double[] topPos = {1.0, 0.0};


    //set to random duck, then keep highest for auton
    volatile SlideState state;
    Object stateMutex = new Object();

    /** Initialization **/
    public SlideController(HardwareMap hardwareMap, Telemetry telemetry, IntakeSide side) {
        verticalServo = hardwareMap.get(Servo.class, "vt");
        horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");

        slideMotor = hardwareMap.get(DcMotor.class, "spool");

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }




    /**
     State setter.
     @param slideState The desired intake state to set to the robot.
     **/

    public void setState(SlideState slideState) {
        synchronized (stateMutex) {
            state = slideState;
        }
        doStateAction();
    }

    /** Accessor for current state **/
    public SlideState getState(){return state;}



    /** Assigns position and motor power to their respective states **/
    private void doStateAction(){

        switch(state){
            case BOTTOM:
                verticalServo.setPosition(bottomPos[0]);
                horizontalServo.setPosition(bottomPos[1]);
                return;

            case MIDDLE:
                verticalServo.setPosition(middlePos[0]);
                horizontalServo.setPosition(middlePos[1]);
                return;

            case TOP:
                verticalServo.setPosition(topPos[0]);
                horizontalServo.setPosition(topPos[1]);
                return;

            case IN:
                slideMotor.setPower(-.3);
                slideMotor.setTargetPosition(0);

        }
    }
    private void populateValues(){
        //TODO: Record actual values.

        values.put(SlideComponents.VERTICAL_SERVO_TOP, 1.0);
        values.put(SlideComponents.VERTICAL_SERVO_MIDDLE, 0.5);
        values.put(SlideComponents.VERTICAL_SERVO_BOTTOM, 0.0);

        values.put(SlideComponents.HORIZONTAL_SERVO_POSITION, 0.5);

        values.put(SlideComponents.DUMPER_SERVO_DUMPED, 0.0);
        values.put(SlideComponents.DUMPER_SERVO_UP, 1.0);

        values.put(SlideComponents.SLIDE_MOTOR_POWER_MOVING, 0.3);
        values.put(SlideComponents.SLIDE_MOTOR_POWER_REST, 0.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED, 0.5);
        values.put()
    }
}

