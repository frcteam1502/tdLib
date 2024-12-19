package org.team1502.configuration.builders.power;

import org.team1502.configuration.builders.Channel;

public class Power {
    /** Power Connector (drain) */
    public static String Vin = "Vin"; // connector
    
    /** Power Connector (source)*/
    public static String Vout = "Vout"; // connector
    
    /** Power Connector "friendly-name" */
    public static String Connector = "Power connector"; 
    
    /** Power Signal (12VDC) */
    public static String Signal = Channel.SIGNAL_12VDC ;
}
