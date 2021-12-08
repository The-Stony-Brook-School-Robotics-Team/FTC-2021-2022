package org.sbs.bears.controller;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorStripController {
    private RevBlinkinLedDriver colorstrip;
    public ColorStripController(HardwareMap hwMap) {
        this.colorstrip = hwMap.get(RevBlinkinLedDriver.class,"colostrip");
        Log.d("ColorStripController","Initialized");
    }
    public void setRed() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void setDarkBlue() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
    }
    public void setOcean() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }
    public void setParty() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
    }
    public void setGreen() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public void setForest() {
        this.colorstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

}
