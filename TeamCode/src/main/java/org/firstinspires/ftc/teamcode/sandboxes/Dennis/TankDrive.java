package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.common.official.auton.v1.config.DriveConstants.WHEEL_RADIUS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TankDrive {

    private DcMotorEx[] motors = new DcMotorEx[4];

    public TankDrive(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        if(lf == null || rf == null || lb == null || rb == null) {
            return;
        } else {
            motors = new DcMotorEx[] {lf, rf, lb, rb};
            motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
            setRunmode(motors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunmode(motors, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setRunmode(DcMotorEx[] motors, DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    private void setTarget(DcMotorEx[] motors, int ticks) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        }
    }

    private void setPower(DcMotorEx[] motors, double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    private boolean isReady() {
        for (DcMotorEx motor : motors) {
            if(motor != null) {
                continue;
            } else {
                return false;
            }
        }
        return true;
    }

    public void move(int inches) {
        if(!isReady()) {
            return;
        } else {
            int ticks = inchesToTicks(inches);
            setTarget(motors, ticks);
            setPower(motors, 0.3);
            setRunmode(motors, DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public boolean isBusy() {
        if(motors[0].isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * @param mult1 adjuster for front left (0.3, 0.4, 0.5)
     * @param mult2 adjuster for right back (1.1, 1.2, 1.3)
     *
     * @example mult1 = 0.5 | to turn left
     * @example mult2 = 1.5 | to turn left
     */
    public void applyLeftMultipliers(double mult1, double mult2) {
        motors[0].setPower(motors[0].getPower() * mult1);
        motors[3].setPower(motors[3].getPower() * mult2);
    }

    /**
     * @param mult1 adjuster for front right (0.5, 0.4, 0.3)
     * @param mult2 adjuster for left back (1.5, 1.4, 1.3)
     *
     * @example mult1 = 1.5 | to turn right
     * @example mult2 = 0.5 | to turn right
     */
    public void applyRightMultipliers(double mult1, double mult2) {
        motors[1].setPower(motors[0].getPower() * mult1);
        motors[2].setPower(motors[3].getPower() * mult2);
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
