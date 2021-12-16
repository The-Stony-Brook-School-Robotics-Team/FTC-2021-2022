package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.enums;

public enum TeleOpRobotStates {
    STOPPED(0), INITIALIZING(1), RUNNING(3);
    private int ident;
    TeleOpRobotStates(int ident) {
        this.ident = ident;
    }
}