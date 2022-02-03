package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * This OpMode is used to Debug the Motor HardwareMap names and Directions.
 * To use, start the OpMode, and press each button to launch each motor
    * A: rb
    * B: lb
    * Y: rf
    * X: lf
 * Change the motor directions to REVERSE as needed to make sure all wheels correspond and spin forward.
 * @author Marc N.
 * @version 2.1.2
 */
public class buttonmotor extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    public static String[] motorNames = new String[]{"rb","lb","rf","lf"}; // rb lb rf lf
    @Override
    public void runOpMode() throws InterruptedException {
        // MARK - Initialization
        // STEP 1: Initialize the motors using the array of names.
        for (int i = 0; i < 4; i++) {
            motors[i] = (hardwareMap.get(DcMotor.class, motorNames[i]));
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // STEP 2: Reverse motors as needed. CHANGE THIS IF IT DOES NOT WORK!
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        // MARK - End of Initialization
        waitForStart();
        // MARK - Start of Run code
        while(opModeIsActive()) {
            // MARK: Start of loop code
            telemetry.addData("RB Enc",motors[0].getCurrentPosition());
            telemetry.update();
            if(gamepad1.a) {
                // RB
                motors[0].setPower(1);
                continue;
            }
            if(gamepad1.b) {
                // LB
                motors[1].setPower(1);
                continue;
            }
            if(gamepad1.y) {
                // RF
                motors[2].setPower(1);
                continue;
            }
            if(gamepad1.x) {
                // LF
                motors[3].setPower(1);
                continue;
            }
            // If not running any motors, put the brakes on!
            stopMotors();
            // MARK - End of loop code
        }
        // MARK - End of RunOpMode
    }

    /**
     * This method simplifies running all motors at a specified power.
     * @param pow The motor power to set all motors at.
     * @author Marc N.
     * @version 1.0
     */
    public void forwardPow(double pow) {
        for (int i = 0; i < 4; i++) {
            // Cycle through all the motors.
            motors[i].setPower(pow);
            // Note - The reversals ensure that setting all the powers positive
            // result in forward motion.
        }
    }

    /**
     * This method simplifies stopping all motors by setting all the powers to zero.
     * @author Marc N.
     * @version 1.0
     */
    public void stopMotors()
    {
        for (int i = 0; i < 4; i++) {
            // Cycle through all the motors.
            motors[i].setPower(0);
        }
    }
    // MARK - End of OpMode Class
}
