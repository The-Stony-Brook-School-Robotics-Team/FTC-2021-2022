package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsTank.*;

public class DriveController {
    public DcMotorEx lf;
    public DcMotorEx rf;
    public DcMotorEx lb;
    public DcMotorEx rb;

    DcMotorEx[] motors;
    DcMotorEx[] leftMotors;
    DcMotorEx[] rightMotors;

    public DriveController(HardwareMap hardwareMap){
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        motors = new DcMotorEx[]{lf, rf, lb, rb};
        leftMotors = new DcMotorEx[]{lf, lb};
        rightMotors = new DcMotorEx[]{rf, rb};

        for(DcMotorEx motor : leftMotors){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        for(DcMotorEx motor : motors){
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setTargetPositionTolerance(5);
            motor.setPositionPIDFCoefficients(10);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void driveTo(int ticks){
        for(DcMotorEx motor : motors){
            motor.setTargetPosition(ticks);
        }
    }

    public void update(){
        if(!rb.isBusy()){
            for(DcMotorEx motor : motors){
                motor.setTargetPosition(motor.getCurrentPosition());
            }
        }
    }



    public static double inchesToEncoderTicks(double inches){
        return (inches * TICKS_PER_REV) / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO);
    }
}
