package org.team1502.configuration.builders;

import java.util.function.Function;

import org.team1502.configuration.CAN.CanInfo;
import org.team1502.configuration.CAN.DeviceType;
import org.team1502.configuration.CAN.Manufacturer;
import org.team1502.configuration.builders.power.Power;

/** https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-introduction.html 
 * The roboRIO is a reconfigurable robotics controller that includes built-in ports for
 *  inter-integrated circuits (I2C), serial peripheral interfaces (SPI), RS232, USB,
 *  Ethernet, pulse width modulation (PWM), an onboard accelerometer,
 *  and a custom electronics port.
*/
public class RoboRIO extends Builder {
    private static final DeviceType deviceType = DeviceType.RobotController;
    public static final String version = "version"; 
    
    /** key to channel number in connector --  */
    public static final String digitalInput = "digitalInput";
    public static final String pwmOutput = "pwmOutput";

    public static final String CLASSNAME = "RoboRIO"; 
    public static final Function<IBuild, RoboRIO> Define = build->new RoboRIO(build);
    public static RoboRIO Wrap(Builder builder) { return builder == null ? null : new RoboRIO(builder.getIBuild(), builder.getPart()); }
    public static RoboRIO WrapPart(Builder builder) { return WrapPart(builder, CLASSNAME); }
    public static RoboRIO WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }
    public RoboRIO(IBuild build) {
        super(build, deviceType, Manufacturer.NI);
        Type(CLASSNAME);
        Name("RoboRIO");
        FriendlyName("roboRIO");
        AddCAN();
        //addPart(Builder.DefineAs(Channel.SIGNAL_CAN)).FriendlyName("CAN port");
        addConnector(Power.Signal, "INPUT").FriendlyName(Power.Connector);
        addConnector(Channel.SIGNAL_ETH, "Ethernet").FriendlyName("Ethernet port");
        addChannel(Power.Signal, "RSL").FriendlyName("RSL port");
        AddDIO();
        AddPWM();
        // I2C
        // SPI
        // RS-232
        CanNumber(0);
    }
    public RoboRIO(IBuild build, Part part) { super(build, part); }
    
    public RoboRIO Version(String version) {
        setValue(RoboRIO.version, version);
        return this;
    }

    Builder CAN() { return getPart(Channel.SIGNAL_CAN); }
    void AddCAN() {
        var can = addPart(Builder.DefineAs(Channel.SIGNAL_CAN)).FriendlyName("roboRIO CAN bus");
    }


    public RoboRIO PWM(Function</*PulseWidthModulatedBus, PulseWidthModulatedBus*/Builder,Builder> fn) {
        fn.apply(PWM());
        return this;
    }
    Builder PWM() { return getPart(Channel.SIGNAL_PWM); }
    void AddPWM() {
        var pwn = addPart(Builder.DefineAs(Channel.SIGNAL_PWM)).FriendlyName("roboRIO PWN port");
        for (int ch = 0; ch < 10; ch++) {
            pwn.addPiece(Channel.Define(Channel.SIGNAL_PWM, CLASSNAME, ch));
        }
    }
    
    public RoboRIO DIO(Function</*DigitalBus, DigitalBus*/Builder,Builder> fn) {
        fn.apply(DIO());
        return this;
    }
    Builder DIO() { return getPart(Channel.SIGNAL_DIO); }
    void AddDIO() {
        var dio = addPart(Builder.DefineAs(Channel.SIGNAL_DIO)).FriendlyName("roboRIO DIO port");
        for (int ch = 0; ch < 10; ch++) {
            dio.addPiece(Channel.Define(Channel.SIGNAL_DIO, CLASSNAME, ch));
        }
    }
    // public RoboRIO DIO(Builder part) {
    //     if (part.hasValue(RoboRIO.digitalInput)) {
    //         return DIO(part.getInt(RoboRIO.digitalInput), part);
    //     }
    //     return this;
    // }
    public RoboRIO DIO(Integer channelNumber, Builder part) {
        updateDioChannel(channelNumber, part);
        return this;
    }
    public void updateDioChannel(Integer channelNumber, Builder part) {
        getChannel(channelNumber).tryConnectToChannel(part);
    }
    public Channel getChannel(int channelNumber) {
        return  Channel.Wrap(DIO().getPiece(channelNumber));
    }

    // public PulseWidthModulatedBus Spark(int channel, String name, String device) {
    //     return this;
    // }

}
