package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

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




    //set to random duck, then keep highest for auton
    volatile SlideState state;
    Object stateMutex = new Object();

    /** Initialization **/
    public SlideController(HardwareMap hardwareMap, Telemetry telemetry) {
        verticalServo = hardwareMap.get(Servo.class, "vt");
        horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");

        slideMotor = hardwareMap.get(DcMotor.class, "spool");

        populateValues();

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }




    /**
     State setter.
     @param slideState The desired slide state to set to the robot.
     **/

    public void setState(SlideState slideState) {
        synchronized (stateMutex) {
            state = slideState;
        }
        doStateAction();
    }

    /** Accessor for current state **/
    public SlideState getState(){return state;}

    public double getServoPosition(){return verticalServo.getPosition();}
    public int getMotorPosition(){return slideMotor.getCurrentPosition();}
    public double getMotorPower(){return slideMotor.getPower();}



    /** Assigns position and motor power to their respective states **/
    private void doStateAction(){
        //TODO: get a motor encoder oh my god
        switch(state){
            case BOTTOM:
                //verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_BOTTOM));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                return;

            case MIDDLE:
                verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_MIDDLE));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                return;

            case TOP:
                verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_TOP));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                return;

            case IN:
                verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_MIDDLE));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue());
                slideMotor.setPower(-values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));


        }
    }
    private void populateValues(){
        //TODO: Record actual values.

        values.put(SlideComponents.VERTICAL_SERVO_TOP, 0.812);
        values.put(SlideComponents.VERTICAL_SERVO_MIDDLE, 0.41);
        values.put(SlideComponents.VERTICAL_SERVO_BOTTOM, 0.1);

        values.put(SlideComponents.HORIZONTAL_SERVO_POSITION, 0.5);

        values.put(SlideComponents.DUMPER_SERVO_DUMPED, 0.0);
        values.put(SlideComponents.DUMPER_SERVO_UP, 1.0);

        values.put(SlideComponents.SLIDE_MOTOR_POWER_MOVING, 1.0);
        values.put(SlideComponents.SLIDE_MOTOR_POWER_REST, 0.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED, 500.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_IN, 25.0);
    }
}

