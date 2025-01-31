package org.team1502.configuration.builders;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;

public class Chassis extends Builder {
  private static final String NAME = "Chassis";
  private static final String chassisLayout = "chassisLayout";

  /** Wheel Base Width (in) */
  private static final String wheelBaseWidth = "wheelBaseWidth";
  /** Wheel Base Length (in) */
  private static final String wheelBaseLength = "wheelBaseLength";
  /** Wheel Base Radius (in) */
  private static final String driveBaseRadius = "driveBaseRadius";

  /** Wheel Diameter (in) *
  private static final String wheelDiameter = "wheelDiameter";
  */
  public static Function<IBuild, Chassis> Define = build->new Chassis(build);
  public static Chassis Wrap(Builder builder) { return builder == null ? null : new Chassis(builder.getIBuild(), builder.getPart()); }
  public static Chassis WrapPart(Builder builder) { return WrapPart(builder, NAME); }
  public static Chassis WrapPart(Builder builder, String partName) { return Wrap(builder.getPart(partName)); }

  public Chassis(IBuild build) { super(build, NAME); }
  public Chassis(IBuild build, Part part) { super(build, part); }
  
  public Chassis Frame(double inches) {
    return this; // TODO? and bumpers for navigation shadow
  }

  public Chassis Square(double inches) {
      Value(chassisLayout, "square");
      Value(wheelBaseWidth, inches);
      Value(wheelBaseLength, inches);
      Value(driveBaseRadius, Math.sqrt((inches/2.0)*(inches/2.0)*2.0));
      return this;
  }
  public Chassis Rectangular(double inchesWidth, double inchesLength) {
      Value(chassisLayout, "rectangular");
      Value(wheelBaseWidth, inchesWidth);
      Value(wheelBaseLength, inchesLength);
      Value(driveBaseRadius, Math.sqrt((inchesWidth/2.0)*(inchesWidth/2.0) + (inchesLength/2.0)*(inchesLength/2.0)));
      return this;
  }

  /** Wheel Diameter (m) *
  public double getWheelDiameter() { return getMeters(wheelDiameter); }
  /** Wheel Diameter (in)  *
  public double WheelDiameter() { return getDouble(wheelDiameter); }
  /** Wheel Diameter *
  public Chassis WheelDiameter(double inches) {
    Value(wheelDiameter, inches);
    return this;
  }
  */
  /** Radius The radius of the drive base in meters. For swerve drive, this is the distance from the center of the robot to the furthest module. For mecanum, this is the drive base width / 2 */
  public double getDriveBaseRadius() { return getMeters(driveBaseRadius); }
  /** radius of drive base in inches */
  public double DriveBaseRadius() { return getDouble(driveBaseRadius); }
  public Chassis DriveBaseRadius(double inches) {
    Value(driveBaseRadius, inches);
    return this;
  }

  
  /**
   * Assumes four modules
   * @param moduleNumber
   * @return offset in meters
   */
  public Translation2d getModuleLocation(int moduleNumber) {
    double halfX = getMeters(wheelBaseWidth)/2;
    if (moduleNumber == 3 || moduleNumber == 4) {// the back row
      halfX = -halfX;
    }
    double halfY = getMeters(wheelBaseLength)/2;
    if (moduleNumber == 2 || moduleNumber == 4) {// the right side
      halfY = -halfY;
    }
    return new Translation2d(halfX, halfY);
  }
}

/*
  

NOTE: THIS ORIENTATION IS ROTATED CCW 90deg

          "FRONT"

    |<-W/2->|

   1,1              1,-1                       ^
   [1]------|-------[2]   ---                  X
    :       :        :     ^                   |
    :       :        :    L/2                  |
    :       :        :     |                   |
    +-------+--------+    ---       < Y -------+
    :       :        :
    :       :        :
    :       :        :
   [3]------|-------[4]
  -1,1             -1,-1
    
  1  1, 1
  2  1,-1
  3 -1, 1
  4 -1,-1  


 */

