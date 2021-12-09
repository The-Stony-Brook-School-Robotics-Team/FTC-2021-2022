package org.firstinspires.ftc.teamcode.sandboxes.Dennis.teleop;

import static org.firstinspires.ftc.teamcode.sandboxes.Dennis.teleop.TeleOp.blinkinLedDriver;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="A - TeleOp")
public class TeleOp extends OpMode {

    // Robot Enum States
    public enum ROBOT_STATE {
        STOPPED,
        IDLE,
        INITIALIZING,
        RUNNING,
        INTERNAL_ERROR
    }

    // Current Robot State
    private static volatile ROBOT_STATE currentState = ROBOT_STATE.STOPPED;
    private Object stateObject = new Object();

    // Blinkin LED Driver
    public static RevBlinkinLedDriver blinkinLedDriver;

    // Motors
    private DcMotorEx lf, rf, rb, lb;

    // Sub Routines
    public static SecondaryRoutine secondaryRoutine;

    @Override
    public void init() {
        synchronized (stateObject) {
            currentState = ROBOT_STATE.IDLE;
        }
        secondaryRoutine = new SecondaryRoutine(hardwareMap);
        if(secondaryRoutine.isInitialized()) {
            secondaryRoutine.start();
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
        switch(currentState) {

            case RUNNING:
                handleWeightedDriving();

            case INTERNAL_ERROR:
                emergencyStop();

        }


    }

    public static ROBOT_STATE returnState() {
        return currentState;
    }

    public void handleWeightedDriving() {
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

    public void emergencyStop() {
        if(lf.isMotorEnabled()) {
            lf.setPower(0);
        }
        if(lb.isMotorEnabled()) {
            lb.setPower(0);
        }
        if(rf.isMotorEnabled()) {
            rf.setPower(0);
        }
        if(rb.isMotorEnabled()) {
            rb.setPower(0);
        }
    }

    public void stop() {
        secondaryRoutine.stop();
    };
}

/**
 * This is the secondary routine which will handle most of the secondary operations for the robot
 */
class SecondaryRoutine {

    // HardwareMap
    private HardwareMap hardwareMap;
    // Internal Routine Controller
    public Thread subroutine;
    // Internals
    private boolean initialized = false;
    private boolean externalinterrupt = false;

    /**
     * SecondaryRoutine is the second thread within the Robot Thread Pool
     * @param hardwareMap the HardwareMap for the robot
     */
    public SecondaryRoutine(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.initialize();
    }

     //                    Add Variables In Here                  \\
    // ------------------------------------------------------------\\






    // ------------------------------------------------------------


    /**
     * Initializes the subroutine
     */
    private boolean initialize() {
        try {
            // Create the thread
            subroutine = new Thread(() -> {
                // TODO: Add Initialization For Subroutine TeleOp


                // Main Loop
                while(!externalinterrupt && initialized) {
                    // add internals here (this is where the code goes!
                    switch(TeleOp.returnState()) {

                        case RUNNING:
                            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);

                        case INTERNAL_ERROR:
                            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

                    }

                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
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
    public int start() {
        if(initialized) {
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
    public int stop() {
        if(isInitialized()) {
            subroutine.checkAccess();
            if(subroutine.isAlive()) {
                subroutine.stop();
                if(subroutine.isAlive()) {
                    return -1;
                } else {
                    return 1;
                }
            }
        } else {
            return 1;
        }
        return -1;
    }

    /**
     * Checks If Subroutine Is Initialized
     * @return true or false
     */
    public boolean isInitialized() {
        if(initialized) {
            return true;
        } else {
            return false;
        }
    }

}