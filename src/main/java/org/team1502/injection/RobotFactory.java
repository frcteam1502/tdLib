package org.team1502.injection;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team1502.TestSupport;
import org.team1502.configuration.annotations.*;
import org.team1502.configuration.factory.RobotBuilder;
import org.team1502.configuration.factory.RobotConfiguration;

public class RobotFactory {
    public static Class<SubsystemInfo> subsystemAnnotation = SubsystemInfo.class;
    private static Class<?> robotClass;
    private static String subsystemPackageName = "frc/robot/subsystems";
    private static String commandPackageName = "frc/robot/commands";

    public static RobotFactory Create() {//} throws ClassNotFoundException {
        return new RobotFactory();
    }

    public static RobotFactory Create(Class<?> robotClass, RobotConfiguration config) {
        RobotFactory.robotClass = robotClass;
        var factory = new RobotFactory(config);
        factory.start();
        return factory;        
    }

    private RobotFactory() {}
    
    private RobotFactory(RobotConfiguration config) {
        jarFile = new File(robotClass.getProtectionDomain().getCodeSource().getLocation().getPath());
        this.configuration = config;
    }
    public RobotConfiguration getRobotConfiguration() { return configuration; }

    private ArrayList<RobotPart> parts = new ArrayList<>();
    private HashMap<String, RobotPart> partMap = new HashMap<>();
    private HashMap<String, SubsystemFactory> subsystemMap = new HashMap<>();
    private HashMap<String, CommandFactory> commandMap = new HashMap<>();
    private List<SubsystemFactory> subsystemFactories;
    private List<CommandFactory> commandFactories;
    private RobotConfiguration configuration;
    private File jarFile;

    private void start() {
        System.out.println("FACTORY: Start");
        gatherSubsystems();
        System.out.println("FACTORY: " + parts.size() + " Subsystems found");
        build();
        for (var part : parts) {
            System.out.println("FACTORY: " + part.getName() + (part.isBuilt() ? " built" : " not built"));
        }
    }

    public void gather() {
        gatherSubsystems();
        gatherCommands();
    }

    public void gatherSubsystems() {
        subsystemFactories = getSubsystemFactories();
        for (SubsystemFactory subsystemFactory : subsystemFactories) {
            System.out.println("Found " + subsystemFactory.getName() + ", enabled=" + (subsystemFactory.isEnabled() ? "true" : "False"));
            subsystemMap.put(subsystemFactory.getName(), subsystemFactory);
            configuration.isSubsystemDisabled(subsystemFactory.getName());
            addPart(subsystemFactory);
        }
    }

    // It should be noted that "commands" aren't necessarily created at the beginning
    // e.g., a drive controller would be the default controller for the drive subsystem
    // but there could be other "task" based commands, triggered by some event
    public void gatherCommands() {
        commandFactories = getCommandFactories();
        for (CommandFactory commandFactory : commandFactories) {
            System.out.println("Found " + commandFactory.getName() + ", enabled=" + (commandFactory.isEnabled() ? "true" : "False"));
            commandMap.put(commandFactory.getName(), commandFactory);
            addPart(commandFactory);
        }
    }

    private RobotPart addPart(RobotPart part) {
        parts.add(part);
        partMap.put(part.getName(), part);
        if (configuration.isDisabled(part.getName())) {
            part.Disable();
        }
        return part;
    }

    private RobotPart needPart(Class<?> partClass) {
        var part = getPart(partClass);
        return (part != null) 
            ? part
            : addPart(new RobotPart(partClass));        
    }

    public RobotPart getPart(Class<?> partClass) {
        return partMap.get(partClass.getName());
    }

    @SuppressWarnings("unchecked")
    public <T> T getInstance(Class<T> partClass) {
        var part = getPart(partClass);
        return part == null ? null : (T)part.getPart();
    }
    
    int systemSize;
    RobotBuilder robotBuilder;
    private void build() {
        systemSize = parts.size(); // just iterate over the subsytems
        addPart(new RobotPart(configuration));

        // a bare builder may need a IBuild-er for ctor, should config do that?
        // ?? side-effects?
        // ?? does this need to be scoped for Sub-RobotConfigurations
        configuration.Build(rb -> robotBuilder = rb);
        Class<?>[] interfaces = robotBuilder.getClass().getInterfaces();
        for (int i = 0; i < interfaces.length; i++) {
            addPart(new RobotPart(robotBuilder, interfaces[i].getClass()));
        }
        buildParts();
    }

    private void buildParts() {
        gatherCommands();
        for (int i = 0; i < systemSize; i++) {
            var part = parts.get(i);
            buildPart(part);
        }
    }

    private Object buildPart(RobotPart part) {
        Object instance = null;
        if (!part.isBuilt() && part.isEnabled()) {
            var dependencies = part.getDependencies();
            Object[] args = new Object[dependencies.length];
            for (int i = 0; i < args.length; i++) {
                var dependency = needPart(dependencies[i]);
                if (dependency.isDisabled()) {
                    part.Disable();
                    break;
                }
                if (!dependency.isBuilt()) {
                    buildPart(dependency);
                }
                args[i] = prepDependency(part, dependency);
            }
            
            // check for null args??
            if (part.isEnabled()) { 
                instance = part.Build(args);
                if (part.hasDefaultCommand()) {
                    var command = (Command)buildPart(needPart(part.getDefaultCommand()));
                    var subsystem = (Subsystem)instance;
                    subsystem.setDefaultCommand(command);
                }
            }
        }
        return instance;
    }

    /** E.g., find a sub-config for a subsystem */
    private Object prepDependency(RobotPart part, RobotPart dependency) {
        var instance = dependency.getPart();
        if (instance instanceof RobotConfiguration) {
            var config = (RobotConfiguration)instance;
            var subsystem = config.findSubsystemConfiguration(part.getName());
            if (subsystem != null) {
                instance = subsystem;
            }
        }
        return instance;
    }

    private List<SubsystemFactory> getSubsystemFactories() {
        Set<Class<Subsystem>> subsystems = getAllSubsystems();
        return subsystems.stream()
            .map(ss->new SubsystemFactory(ss))
            .toList();
    }

    private List<CommandFactory> getCommandFactories() {
        Set<Class<Command>> commands = getAllCommands();
        return commands.stream()
            .map(ss->new CommandFactory(ss))
            .toList();
    }
    

    @SuppressWarnings("unchecked")
    private Set<Class<Subsystem>> getAllSubsystems() {
        
        try {
            ArrayList<String> classes = new ArrayList<>();
            ClassLoader loader = getClassLoader();
            if(jarFile.isFile()) {  // Run with JAR file
                getClasses(subsystemPackageName, classes);
            } else {
                findClassesIn(subsystemPackageName, loader, classes);
            }
            
            return classes.stream()
                .map(name -> getClass(loader, name))
                .filter(candidate -> SubsystemFactory.isSubsystem(candidate))
                .map(candidate -> (Class<Subsystem>)candidate)
                .collect(Collectors.toSet());

        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Collections.EMPTY_SET;
        }
    }

    @SuppressWarnings("unchecked")
    private Set<Class<Command>> getAllCommands() {
        try {
            ArrayList<String> classes = new ArrayList<>();
            ClassLoader loader = getClassLoader();
        
            if(jarFile.isFile()) {  // Run with JAR file
                getClasses(commandPackageName, classes);
            } else {
                findClassesIn(commandPackageName, loader, classes);
            }
            return classes.stream()
                .map(name -> getClass(loader, name))
                .filter(line -> CommandFactory.isSubsystem(line))
                .map(line -> (Class<Command>)line)
                .collect(Collectors.toSet());
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Collections.EMPTY_SET;
        }
    }
 

    private Class<?> getClass(ClassLoader loader, String name) {
        try {
            return loader.loadClass(name);
        } catch (ClassNotFoundException e) {
            System.err.println("FACTORY :: unable to load: " + name);
            return null;
        }
    }

    private void findClassesIn(String folder, ClassLoader loader, ArrayList<String> classes) throws IOException {
        InputStream stream = loader.getResourceAsStream(folder);
        if (stream == null) return;
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));
        while (reader.ready()) {
            String line = reader.readLine();
            if (line.endsWith(".class")) {
                String className = line.substring(0, line.length()-6);
                String packageName = folder.replaceAll("[/]", ".");
                classes.add(packageName + "." + className);
            } else {
                findClassesIn(folder + "/" + line, loader, classes);            
            }
        }        
    }

    private void getClasses(String path, ArrayList<String> classes) throws IOException {
        if(jarFile.isFile()) {  // Run with JAR file
            final JarFile jar = new JarFile(jarFile);
            final Enumeration<JarEntry> entries = jar.entries(); //gives ALL entries in jar
            while(entries.hasMoreElements()) {
                final String name = entries.nextElement().getName();
                if (name.startsWith(path + "/") && name.endsWith(".class")) {
                    classes.add(name.substring(0, name.length()-6)
                                    .replaceAll("[/]", "."));
                }
            }
            jar.close();
        }
    }

    ClassLoader getClassLoader() throws IOException {
        if (TestSupport.getIsTesting()) {
            return getTestingClassLoader();
        } else {
            return robotClass.getClassLoader();
        }
    }
    ClassLoader getTestingClassLoader() throws IOException {
        var clazzPackage = robotClass.getPackage();
        var clazzPackageName = clazzPackage.getName();
        var clazzPath = clazzPackageName.replace('.', '/');
        String mainUrl = "";
        //return robotClass.getClassLoader();
        
        var resources = robotClass.getClassLoader().getResources(clazzPath);
        while (resources.hasMoreElements()) {
            var url = resources.nextElement();
            System.out.println(url);
            if (url.getPath().contains("/main/")) {
                mainUrl = url.toExternalForm() + "/";
            }
        }
        return new URLClassLoader(new URL[]{new URL(mainUrl)});

    }

}
