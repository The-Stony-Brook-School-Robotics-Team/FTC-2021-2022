package org.firstinspires.ftc.teamcode.common.official.auton.v1.movement;

import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.WHEEL_RADIUS;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants;

public class DriveTrain {

    // motors
    private DcMotorEx[] motors = new DcMotorEx[4];

    /**
     * DriveTrain Constructor
     * @param lf
     * @param rf
     * @param lb
     * @param rb
     */
    public DriveTrain(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        // set direction
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motors[0] = lf;
        this.motors[1] = rf;
        this.motors[2] = lb;
        this.motors[3] = rb;

        // runmode (stop, reset + run)
        // this.runmode(this.motors, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set tolerance
        tolerance(this.motors, DriveConstants.positionTolerance);
    }

    /**
     * Set the runmode of each motor (ex 1)
     * @param motors motor list
     * @param r1 first runmode to set
     */
    private void runmode(DcMotorEx[] motors, DcMotorEx.RunMode r1) {
        for(DcMotorEx motor : motors) {
            motor.setMode(r1);
        }
    }

    /**
     * Set the runmode of each motor (ex 2)
     * @param motors motor list
     * @param r1 first runmode to set
     * @param r2 second runmode to set
     */
    private void runmode(DcMotorEx[] motors, DcMotorEx.RunMode r1, DcMotorEx.RunMode r2) {
        for(DcMotorEx motor : motors) {
            motor.setMode(r1);
            motor.setMode(r2);
        }
    }

    /**
     * Sets the PID tolerance
     * @param motors motor list
     * @param tolerance the tolerance in ticks you want to set
     */
    private void tolerance(DcMotorEx[] motors, int tolerance) {
        for(DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    /**
     * Makes the motors run to a position
     * @param inches the amount in inches you want to move
     */
    public void move(int inches) {
        for(DcMotorEx motor : this.motors) {
            Log.d("DRIVETRAIN DENNIS (Target Tick)", String.valueOf(motor.getCurrentPosition() + inchesToTicks(inches)));
            motor.setTargetPosition(motor.getCurrentPosition() + inchesToTicks(inches));
            motor.setPower(0.1);
        }
        this.runmode(this.motors, DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Converts ticks to inches
     * @param ticks amount in ticks
     * @return amount in inches
     */
    private int ticksToInches(int ticks) {
        return (int) (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV);
    }

    /**
     * Converts inches to ticks
     * @param inches amount in inches
     * @return amount in ticks
     */
    private int inchesToTicks(int inches) {
        return (int) ((inches * TICKS_PER_REV) / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO));
    }


}
