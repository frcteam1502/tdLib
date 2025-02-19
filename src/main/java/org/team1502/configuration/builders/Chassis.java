package org.team1502.configuration.builders;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.measure.Distance;

public class Chassis extends Builder {
    public static final String NAME = "Chassis";
    private static final String chassisLayout = "chassisLayout";

    /** Wheel Base Width (m) */
    private static final String trackWidth = "trackWidth";
    /** Wheel Base Length (m) */
    private static final String wheelbaseLength = "wheelbaseLength";
    /** Wheel Base Radius (m) */
    private static final String driveBaseRadius = "driveBaseRadius";

    /**
     * Wheel Diameter (m) *
     * private static final String wheelDiameter = "wheelDiameter";
     */
    public static Function<IBuild, Chassis> Define = build -> new Chassis(build);

    public static Chassis Wrap(Builder builder) {
        return builder == null ? null : new Chassis(builder.getIBuild(), builder.getPart());
    }

    public static Chassis WrapPart(Builder builder) {
        return WrapPart(builder, NAME);
    }

    public static Chassis WrapPart(Builder builder, String partName) {
        return Wrap(builder.getPart(partName));
    }

    public Chassis(IBuild build) {
        super(build, NAME);
    }

    public Chassis(IBuild build, Part part) {
        super(build, part);
    }

    /** Full frame including bumpers */
    public Chassis Frame(Distance frontWidth, Distance sideLength) {
        return this;
    }

    /** Full frame including bumpers */
    public Chassis Frame(Distance sideLength) {
        return this; // TODO? add bumpers for navigation shadow
    }

    private void setWheelPlacement(Distance track, Distance wheelbase) {
        double trackMeters = track.in(Meters);
        double wheelbaseMeters = wheelbase.in(Meters);
        Value(trackWidth, trackMeters);
        Value(wheelbaseLength, wheelbaseMeters);
        Value(driveBaseRadius, Math
                .sqrt((trackMeters / 2.0) * (trackMeters / 2.0) + (wheelbaseMeters / 2.0) * (wheelbaseMeters / 2.0)));

    }

    /**
     * kinematics of wheel placement
     * 
     * @param track the track width and the wheelbase
     */
    public Chassis Square(Distance track) {
        Value(chassisLayout, "square");
        setWheelPlacement(track, track);
        return this;
    }

    /**
     * kinematics of wheel placement
     * 
     * @param track     the track width (distance between left and right wheels)
     * @param wheelbase the wheelbase (distance between front and rear wheels)
     * @return
     */
    public Chassis Rectangular(Distance track, Distance wheelbase) {
        Value(chassisLayout, "rectangular");
        setWheelPlacement(track, wheelbase);
        return this;
    }

    /**
     * Radius The radius of the drive base in meters. For swerve drive, this is the
     * distance from the center of the robot to the furthest module. For mecanum,
     * this is the drive base width / 2
     */
    public double getDriveBaseRadius() {
        return getDouble(driveBaseRadius);
    }

    public Chassis DriveBaseRadius(Distance radius) {
        Value(driveBaseRadius, radius.in(Meters));
        return this;
    }

    /**
     * Assumes four modules
     * 
     * @param moduleNumber
     * @return offset in meters
     */
    public Translation2d getModuleLocation(int moduleNumber) {
        double halfX = getMeters(trackWidth) / 2;
        if (moduleNumber == 3 || moduleNumber == 4) {// the back row
            halfX = -halfX;
        }
        double halfY = getMeters(wheelbaseLength) / 2;
        if (moduleNumber == 2 || moduleNumber == 4) {// the right side
            halfY = -halfY;
        }
        return new Translation2d(halfX, halfY);
    }

    public MecanumDriveKinematics getMecanumDriveKinematics() {
        return new MecanumDriveKinematics(
                getModuleLocation(1),
                getModuleLocation(2),
                getModuleLocation(3),
                getModuleLocation(4));
    }

}

/*
  

NOTE: THIS ORIENTATION IS ROTATED CCW 90deg

          "FRONT"

    |<-W/2->|

   1,1              1,-1                       ^
   [1]------|-------[2]   --- ---              X
    :       :        :    /\                   |
    :       :        :    L    w               |
    :       :        :    /    h               |
    :       :        :    2    e               |
    :       :        :    \/   e               |
    +-------+--------+    ---  l    < Y -------+
    :       :        :         b
    :       :        :         a
    :       :        :         s
    :       :        :         e
    :       :        :         
   [3]------|-------[4]       ---
  -1,1             -1,-1

   |<- track width ->| 
  
  1  1, 1
  2  1,-1
  3 -1, 1
  4 -1,-1  


 */
