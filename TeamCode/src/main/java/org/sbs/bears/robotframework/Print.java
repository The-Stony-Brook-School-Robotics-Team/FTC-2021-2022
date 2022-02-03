/****************************************************************
 * Copyright (c) 2022 Michael Mihaley LLC.
 * All rights reserved.
 ***************************************************************/


package org.sbs.bears.robotframework;

import android.util.Log;

/**
 * @Author Michael J. Mihaley
 * @License MIT
 */
public class Print {
    /**
     * Prints.
     * @param o The object to print.
     */
    public static void print(Object o){
        Log.d("Print: ", String.valueOf(o));
    }

}
