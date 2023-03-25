package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 5;
    public static final int kRightMotor2Port = 2;
    public static final double kTrackwidthMeters = 0.6858;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        //GR: 22:48
        //new: 12:36
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.1452;
    public static final double kvVoltSecondsPerMeter = 2.8382;
    public static final double kaVoltSecondsSquaredPerMeter = 0.20793;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
    public static final double kTurnToleranceDeg = 0; //how much we can turn - need to set it up
    public static final double kTurnRateToleranceDegPerS = 5; //rate at which it can turn per second - need to set up 
    public static final double kMaxTurnRateDegPerS = 0; //set this
    public static final double kMaxTurnAccelerationDegPerSSquared = 0; //set this
    public static final double kMaxSpeedMetersPerSecond = 0; //set this
    public static final double kMaxAccelerationMetersPerSecondSquared = 0; //set this

    public static final double kP = 3.2181;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kcircumference = kWheelDiameterMeters * Math.PI;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxVoltageConstraint = 7;
    public static final PIDController leftPID = new PIDController(DriveConstants.kP, 0, 0);
    public static final PIDController rightPID = new PIDController(DriveConstants.kP, 0, 0);
    
    public static final TrajectoryConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
        kMaxVoltageConstraint);

    public static final TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond, 
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
    // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final RamseteController controller = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(0, 0, 0);
  }

  public static final class BalanceConstants {
    public static double kOffBalanceAngleThresholdDegrees = 0;
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CalibrationConstants {
    public static final double testAngle = 180;
    public static final double testSetpoint = 100;
    public static final double testVelocity = 25;

  }

  public static final class CascadeConstants {
    public static final int kCascadeMotor1Port = 6;
    public static final int kCascadeMotor2Port = 7;
    public static final double stallCurrent = 15;
    public static final double nVolt = 12;
    public static final double testSetpoint = 100;
    public static final double testVelocity = 25;
    public static final double kP = 3.2181;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kstage0 = 0;
    public static final double kstage1 = -5.8826;
    public static final double kstage2 = -13.17;
    public static final double kstage3 = -19.04;
    public static final double rotToTickConver = 0;
  }

  public static final class IntakeConstants {
    public static double CONVEYOR_SPEED = .5;
    public static double INTAKE_SPEED = -0.5;
    public static double INDEXER_SPEED = .7;  
  }

  public static final class LimelightConstants {
    public static final double tx = 0;
    public static final double ty = 0;
    public static final double tv = 0;
    public static final double ta = 0;
    public static final double tl = 0;
    public static final double cl = 0;
    public static final double tshort = 0;
    public static final double tlong = 0;
    public static final double thor = 0;
    public static final double tvert = 0;
    public static final double tpipe = 0;
    public static final double tjson = 0;
    public static final double tclass = 0;
    public static final double speed = 0;

    public static final double GOAL_HEIGHT = 0;
    public static final double LIMELIGHT_HEIGHT = 0;
    public static final double LIMELIGHT_MOUNT_ANGLE = 0;

  }
}