package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;

public interface ConstantFile {
    public HashMap<? extends Enum, Object>  getAllConstants();
    public void setAllConstants(HashMap<? extends Enum, Object> newData);
}
