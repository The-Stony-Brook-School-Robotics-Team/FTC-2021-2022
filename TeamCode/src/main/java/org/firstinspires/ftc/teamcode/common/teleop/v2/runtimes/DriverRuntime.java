package org.firstinspires.ftc.teamcode.common.teleop.v2.runtimes;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.teleop.v2.enums.TeleOpStatus;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class DriverRuntime {

    /**
     * Vars
     */
    private Gamepad gamepad = null;
    private TeleOpStatus teleOpStatus = null;
    private SampleTankDrive drive = null;

    public String taq = "GamepadRuntme";

    public DriverRuntime(SampleTankDrive drive, Gamepad gamepad, TeleOpStatus status) {
        if(gamepad != null && status != null && drive != null) {
            this.gamepad = gamepad;
            this.teleOpStatus = status;
            this.drive = drive;
        }
    }

    /**
     * Checks for null variables in the runtime
     * @return the status of the gamepad runtime
     */
    public boolean ready() {
        if(this.gamepad != null & this.teleOpStatus != null && drive != null) {
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
            if(this.teleOpStatus == TeleOpStatus.paused) {
                int i = 1 + 1;
            } else {
                // tank drive
                drive.setMotorPowers(
                        -gamepad.left_stick_x + gamepad.right_stick_x,
                        -gamepad.left_stick_x - gamepad.right_stick_x
                );
                drive.update();
            }
        }
    });
}
