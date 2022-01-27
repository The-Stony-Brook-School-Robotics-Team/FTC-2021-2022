package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.debug;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

@Config
@TeleOp(name="U - Encoder Debugger", group="default")
public class EncoderDebugger extends LinearOpMode {

    /**
     * Public Configuration
     */
    public static boolean DEBUG_THREE_WHEEL = true;
    public static String LEFT_ENCODER_NAME = "left";
    public static String RIGHT_ENCODER_NAME = "right";
    public static String CENTER_ENCODER_NAME = "center";
    /**
     * Internal Configuration
     */
    private static EncoderDebugEx[] DEBUG_ENCODERS; // wrapped encoders | non direct
    private static DcMotorEx[] REGULAR_ENCODERS; // unwrapped encoders | direct access
    private static boolean initialized = false; // internally set | internal
    private static boolean interruptSignal = false; // programs interrupt signal for the main program
    /**
     * Internal Flag Setting
     */
    public static int MAX_FLAGS = 5;
    private static int CURRENT_FLAGS = 0;
    private static boolean INTERNAL_THREW_FLAG;
    private static boolean INTERNAL_NEW_FLAG;
    private DEBUG_FLAGS LAST_FLAG;
    private DEBUG_FLAGS NEW_FLAG;
    /**
     * Actual flags
     */
    enum DEBUG_FLAGS {
        DEFAULT_ERROR(1),
        WORKER_ERROR(2),
        INTERNAL_ERROR(3),
        NAME_ERROR(4),
        INITIALIZE_ERROR(5),
        FLAG_HANDLER_ERROR(6),
        FLAG_REMOVAL_ERROR(7);
        private int code;
        DEBUG_FLAGS(int code) { this.code = code; }
        public int getCode() { return this.code; }
    }
    /**
     * Actual Flag List, Accessible By The OpMode
     */
    private ArrayList<DEBUG_FLAGS> FLAGS = new ArrayList<DEBUG_FLAGS>();
    private HashMap<DEBUG_FLAGS, Telemetry.Item> FLAG_ITEMS = new HashMap<>();
    /**
     * Flag Thread
     * Manages flags and throws if required
     */
    private FlagHandlerThreadEx INTERNAL_FLAG_HANDLER = new FlagHandlerThreadEx();
    /**
     * Debug Wheels
     */
    enum DEBUG_WHEELS {
        LEFT(1),
        RIGHT(2),
        CENTER(3);
        private int identifier;
        DEBUG_WHEELS(int identifier) {
            this.identifier = identifier;
        }
    }
    /**
     * Array To Access Which Wheels Need To Be Displayed
     */
    // TODO: When debugging certain wheels, add them into the debug wheels,
    //  keep in mind adding more than 3 and more than the same class is stupid so don't do it???
    private DEBUG_WHEELS[] DEBUG_THESE_WHEELS = new DEBUG_WHEELS[] { DEBUG_WHEELS.LEFT, DEBUG_WHEELS.RIGHT };


    @Override
    public void runOpMode() throws InterruptedException {
        // Starts The Internal Flag Handler
        INTERNAL_FLAG_HANDLER.start();
        try {
            if(!initialized) {
                initialize(hardwareMap);
            }
        } catch (Exception ex) {
            ex.printStackTrace();
            appendFlag(DEBUG_FLAGS.INTERNAL_ERROR);
        }

        waitForStart();

        // TODO: Add An Actual Worker For Telemetry
        while(!isStopRequested() && opModeIsActive() && !interruptSignal) {
            telemetry.addData("left", DEBUG_ENCODERS[0].getPosition());
            telemetry.addData("right", DEBUG_ENCODERS[1].getPosition());
            telemetry.addData("center", DEBUG_ENCODERS[2].getPosition());
            telemetry.update();
        }

    }

    /**
     * Initializes Encoders and Encoder Lists
     * @param hardwareMap hardwareMap for the robot | cannot be null or will throw flag
     */
    public void initialize(@NonNull HardwareMap hardwareMap) {
        try {
            // Flood Internal Encoder Arrays For Access
            for(DEBUG_WHEELS DEBUG_WHEEL : DEBUG_THESE_WHEELS) {
                switch (DEBUG_WHEEL) {

                    case LEFT:
                        DcMotorEx left = hardwareMap.get(DcMotorEx.class, LEFT_ENCODER_NAME);
                        DEBUG_ENCODERS[0] = new EncoderDebugEx(left);
                        REGULAR_ENCODERS[0] = left;
                        break;

                    case RIGHT:
                        DcMotorEx right = hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER_NAME);
                        DEBUG_ENCODERS[1] = new EncoderDebugEx(right);
                        REGULAR_ENCODERS[1] = right;
                        break;

                    case CENTER:
                        DcMotorEx center = hardwareMap.get(DcMotorEx.class, CENTER_ENCODER_NAME);
                        DEBUG_ENCODERS[2] = new EncoderDebugEx(center);
                        REGULAR_ENCODERS[3] = center;

                }
            }
            initialized = true;
        } catch(Exception ex) {
            ex.printStackTrace();
            appendFlag(DEBUG_FLAGS.INITIALIZE_ERROR);
            initialized = false;
        }
    }

    /**
     * Allows the program to add internal flags to the flag list
     * @param flag flag to add
     */
    public void appendFlag(DEBUG_FLAGS flag) {
        if(FLAGS.size() == MAX_FLAGS) {
            telemetry.addLine("REACHED MAX FLAGS");
        } else {
            FLAGS.add(flag);
            CURRENT_FLAGS++;
        }
    }

    public void removeFlag(DEBUG_FLAGS flag) {
        try {
            FLAGS.remove(flag);
            CURRENT_FLAGS--;
        } catch (Exception ex) {
            ex.printStackTrace();
            appendFlag(DEBUG_FLAGS.FLAG_REMOVAL_ERROR);
        }

    }

    /**
     * Flag Handler
     */
    class FlagHandlerThreadEx {

        /**
         * Internally Wrapped Thread
         */
        private Thread internalThread;

        /**
         * Default Flag Handler Thread
         */
        public FlagHandlerThreadEx() {
            Thread internalFlagHandler = new Thread(this::FLAG_HANDLER_ROUTINE);
            internalThread = internalFlagHandler;
        }

        /**
         * Thread Handler Ex Creation | Optional
         * this is here if you have your own flag handler, in that case flags wil be handled by your thread instead
         * @param thread thread to manage your flags
         */
        public FlagHandlerThreadEx(Thread thread) {
            this.internalThread = thread;
        }

        /**
         * Starts Flag Handler
         */
        public void start() {
            internalThread.start();
        }

        /**
         * Stops Flag Handler
         */
        public void stop() {
            internalThread.stop();
        }

        /**
         * Internal Flag Handler Routine
         */
        private void FLAG_HANDLER_ROUTINE() {
            if(INTERNAL_NEW_FLAG) {
                throwLastFlag();
            } else {
                if(INTERNAL_THREW_FLAG && !INTERNAL_NEW_FLAG) {
                    INTERNAL_THREW_FLAG = false;
                }
            }
        }
    }

    /**
     * Throws Last Flag | Used by Flag Handler Thread
     */
    public void throwLastFlag() {
        try {
            if(CURRENT_FLAGS == MAX_FLAGS) {
                // TODO: Remove The First Flag Added, Shift, And Append Another On The End
            }
            Telemetry.Line newLine = telemetry.addLine("Flag: " + NEW_FLAG + " Code: " + NEW_FLAG.getCode());
            telemetry.update();
            LAST_FLAG = NEW_FLAG;
            INTERNAL_NEW_FLAG = false;
            INTERNAL_THREW_FLAG = true;
            interruptSignal = true;
        } catch (Exception ex) {
            ex.printStackTrace();
            appendFlag(DEBUG_FLAGS.FLAG_HANDLER_ERROR);
            INTERNAL_NEW_FLAG = false;
            INTERNAL_THREW_FLAG = false;
        }
    }
}

/**
 * Wrapper Class for DcMotorEx
 */
class EncoderDebugEx {

    private DcMotorEx encoder;

    public EncoderDebugEx(DcMotorEx encoder) {
        this.encoder = encoder;
    }

    public String getConnectionInfo() {
        return encoder.getConnectionInfo();
    }

    public DcMotorController getController() {
        return encoder.getController();
    }

    public int getPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentAmps() {
        return encoder.getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrentMiliAmps() {
        return encoder.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public DcMotorEx getEncoder() {
        return this.encoder;
    }
}

