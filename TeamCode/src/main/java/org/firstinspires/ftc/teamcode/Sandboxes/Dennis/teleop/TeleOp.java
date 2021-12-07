package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="A - TeleOp")
public class TeleOp extends OpMode {

    // Robot Enum States
    private static enum ROBOT_STATE {
        STOPPED,
        IDLE,
        INITIALIZING,
        RUNNING
    }

    // Current Robot State
    private static volatile ROBOT_STATE currentState = ROBOT_STATE.STOPPED;
    private Object stateObject = new Object();

    // Blinkin LED Driver
    private static RevBlinkinLedDriver blinkinLedDriver;

    // Motors
    private DcMotorEx lf, rf, rb, lb;


    @Override
    public void init() {
        synchronized (stateObject) {
            currentState = ROBOT_STATE.IDLE;
        }
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }



    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }
}

/**
 * This is the secondary routine which will handle most of the secondary operations for the robot
 */
class SecondaryRoutine {

    // HardwareMap
    private HardwareMap hardwareMap;
    // Internal Routine Controller
    private Thread subroutine;
    // Internals
    private boolean initializedRoutine = false;
    private boolean externalinterrupt = false;

    /**
     * SecondaryRoutine is the second thread within the Robot Thread Pool
     * @param hardwareMap the HardwareMap for the robot
     */
    public SecondaryRoutine(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.initializeRoutine();
    }

     //                    Add Variables In Here                  \\
    // ------------------------------------------------------------\\






    // ------------------------------------------------------------


    /**
     * Initializes the subroutine
     */
    public boolean initializeRoutine() {
        try {
            // Create the thread
            subroutine = new Thread(() -> {
                while(!externalinterrupt && initializedRoutine) {
                    // add internals here (this is where the code goes!
                }
            });
            // Return
            return true;
        } catch(Exception ex) {

            return false;
        }

    }

    /**
     * Enable Internal Thread
     * @return status
     */
    public int enable() {
        if(initializedRoutine) {
            subroutine.start();
            return 1;
        } else {
            return -1;
        }
    }

    /**
     * Disable Internal Thread
     * @return status
     */
    public int disable() {
        if(initializedRoutine) {
            subroutine.stop();
            return 1;
        } else {
            return -1;
        }
    }

}