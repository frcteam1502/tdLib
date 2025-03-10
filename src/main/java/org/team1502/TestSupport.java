package org.team1502;

public class TestSupport {
    public static void setIsTesting(boolean isTestEnvironment) {
        System.setProperty("test.env", isTestEnvironment ? "True" : "False");
    }
    public static boolean getIsTesting() {
        return System.getProperty("test.env") == "True";
    }
    
}
