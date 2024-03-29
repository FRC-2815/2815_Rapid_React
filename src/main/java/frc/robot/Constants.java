/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DriveConstants {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        
        public static double ksVolts = 0.20407;
        public static double kvVoltSecondsPerMeter = 2.8133;
        public static double kaVoltSecondsSquaredPerMeter = 0.44351;

        // Example value only - as above, this must be tuned for your drive!
        public static double kPDriveVel = 0.012745;

        public static final double kTrackwidthMeters = Units.inchesToMeters(18);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        
        // Added from WPI's DifferentialDrive voltage system
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter);
        
        // Added from WPI's DifferentialDrive voltage system
        public static final PIDController kLeftPIDController = new PIDController(kPDriveVel, 0, 0);
        public static final PIDController kRightPIDController = new PIDController(kPDriveVel, 0, 0);

        public static final double kMaxSpeedMetersPerSecond = 3; //Originally 3m/s
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //Originally 3m/s/s
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; //TEMPLATE VALUE

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        //Custom constants for NEOs and SparkMax's
        public static final double kMaxRPM = 5700;
        public static final double kWheelDiameter = Units.inchesToMeters(6);
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kMotorGearsToWheelGears = 10.71;
        public static final double kRevolutionsToMeters = Math.PI * kWheelDiameter / kMotorGearsToWheelGears;
        public static final double kRPMtoMetersPerSecond = Math.PI * kWheelDiameter / (60 * kMotorGearsToWheelGears);
        
        public static final boolean kGyroReversed = true;
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 4;
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }
    
    public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3; //Originally 3 m/s/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //Originally 3 m/s/s

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    //susan bananas
    }
}