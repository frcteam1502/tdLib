package org.team1502.injection;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLClassLoader;

import org.junit.jupiter.api.Test;
import org.team1502.TestSupport;

public class RobotFactoryTest {
    // As the classpath shows, 'test' and 'main' are both included. I do not know how to search
    // such that I can find all the classes and filter out the test ones.

    // This test is to determine a scheme for when doing robot-testing RobotFactory can find the
    // subsystem classes, etc., in 'main'. The existing factory would ONLY find 'test' classes in
    // e.g., \test\frc\robot\subsystems instead of the desired \main\frc\robot\subsystems

    /* CLASSPATH "
    \tdLib\build\resources\test;
    c:\Users\jonat\source\frcteam\libraries\tdLib\build\classes\java\test;
    \tdLib\build\resources\main;
    c:\Users\jonat\source\frcteam\libraries\tdLib\build\classes\java\main;
    \tdLib\bin;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\cscore\cscore-java\2025.2.1\cscore-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\wpilibj\wpilibj-java\2025.2.1\wpilibj-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\apriltag\apriltag-java\2025.2.1\apriltag-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\cameraserver\cameraserver-java\2025.2.1\cameraserver-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\epilogue\epilogue-runtime-java\2025.2.1\epilogue-runtime-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\org\ejml\ejml-fsparse\0.43.1\ejml-fsparse-0.43.1.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\wpinet\wpinet-java\2025.2.1\wpinet-java-2025.2.1.jar;
    c:\Users\Public\wpilib\2025\maven\com\fasterxml\jackson\core\jackson-annotations\2.15.2\jackson-annotations-2.15.2.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\wpimath\wpimath-java\2025.2.1\wpimath-java-2025.2.1.jar;
    c:\Users\jonat\.gradle\caches\modules-2\files-2.1\com.revrobotics.frc\REVLib-java\2025.0.2\4fc59290454689badadbb04cdc40ee824a2fc116\REVLib-java-2025.0.2.jar;
    c:\Users\Public\wpilib\2025\maven\edu\wpi\first\thirdparty\frc2025\opencv\opencv-java\4.10.0-3\opencv-java-4.10.0-3.jar;
    c:\Users\Public\wpilib\2025\maven\org\ejml\ejml-ddense\0.43.1\ejml-ddense-0.43.1.jar;
    c:\Users\Public\wpilib\2025\maven\org\ejml\ejml-cdense\0.43.1\ejml-cdense-0.43.1.jar;
    etc    
    */
    // The injection methods find the "wrong" package. Are there two "org.team1502.injection"?
    // if you try to find a class but there is the same class in the test directory
    // the you do not get the class under test, but the test itself.
    // E.g this class and RobotFactory are in the same package (kinda?)

    @Test
    public void findClassesInMain() throws IOException  {
        TestSupport.setIsTesting(true);
        
        // When JUnit testing the 'test' and 'main' are commingled and it looks like 'test' gets precedence
        var mainClassLoader = ClassLoader.getSystemClassLoader();
        var clazz = TestSupport.class; // get a reference to a class in 'main' (e.g., 'Robot' in frc program)
        var clazzPackage = clazz.getPackage();
        var clazzPackageName = clazzPackage.getName();
        var clazzPath = clazzPackageName.replace('.', '/');
        String mainUrl = "";
        var resources = mainClassLoader.getResources(clazzPath);
        while (resources.hasMoreElements()) {
            var url = resources.nextElement();
            System.out.println(url);
            if (url.getPath().contains("/main/")) {
                mainUrl = url.toExternalForm() + "/";
            }
        }

        var folder = "injection/";

        // FAIL
        // var loader2 = new URLClassLoader(new URL[]{new URL(mainUrl)}); mainUrl needs a '/'
        // dumpClasses(loader2, "/injection");
        // dumpClasses(loader2, "/injection/");

        var loader3 = new URLClassLoader(new URL[]{new URL(mainUrl)}); // mainUrl ends with '/'
        dumpClasses(loader3, "injection"); // fail: folder also needs a '/' to get a list of files
        dumpClasses(loader3, folder); // good
        dumpClasses(loader3, "configuration/"); //good


        // var resources = loader.getResources("org/team1502/injection/RobotFactory.class");
        // while (resources.hasMoreElements()) {
        //     System.out.println(resources.nextElement());
        // }
        // //loader.getSystemClassLoader()
        // InputStream stream = loader.getResourceAsStream(folder);
        // if (stream == null) return;
        // BufferedReader reader = new BufferedReader(new InputStreamReader(stream3));
        // while (reader.ready()) {
        //     String line = reader.readLine();
        //     System.out.println(line);
        //     if (line.endsWith(".class")) {
        //         String packageName = folder.replaceAll("[/]", ".");
        //     }
        // }        

     
    }
    
    void dumpClasses(ClassLoader loader, String folder) throws IOException  {
        System.out.println("");
        System.out.println(folder + "  ...");
        InputStream stream = loader.getResourceAsStream(folder);
        if (stream == null) return;
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));
        while (reader.ready()) {
            String line = reader.readLine();
            System.out.println(line);
        }        

    }
}
