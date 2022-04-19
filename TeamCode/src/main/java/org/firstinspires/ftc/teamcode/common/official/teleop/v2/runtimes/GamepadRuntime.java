package org.firstinspires.ftc.teamcode.common.official.teleop.v2.runtimes;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.official.teleop.v2.enums.TeleOpStatus;

public class GamepadRuntime {

    /**
     * Vars
     */
    private Gamepad gamepad = null;
    private TeleOpStatus teleOpStatus = null;
    private Boolean secondary = null;

    public String taq = "GamepadRuntme";

    /**
     *
     * @param gamepad
     * @param status
     */
    public GamepadRuntime(Gamepad gamepad, TeleOpStatus status, Boolean secondary) {
        if(gamepad != null && status != null && secondary != null) {
            this.gamepad = gamepad;
            this.teleOpStatus = status;
            this.secondary = secondary;
        }
    }

    /**
     * Checks for null variables in the runtime
     * @return the status of the gamepad runtime
     */
    public boolean ready() {
        if(this.gamepad != null & this.teleOpStatus != null && this.secondary != null) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Starts the runtime thats in the class
     */
    public void start() {
        if(!runtime.isAlive()) {
            runtime.start();
        }
    }

    /**
     * Stops the runtime thats in the class
     */
    public void stop() {
        if(runtime.isAlive()) {
            runtime.interrupt();
        }
    }

    /**
     * Main runtime
     */
    private Thread runtime = new Thread(() -> {
        if(!ready()) {
            Log.d(this.taq, "A variable in the class was null");
        }
        // while the opmode isn't stopped
        while(this.teleOpStatus != TeleOpStatus.stopped) {
            if(this.secondary) {
                // secondary controls go here


            } else {
                // primary controls go here





            }


        }
    });




}
