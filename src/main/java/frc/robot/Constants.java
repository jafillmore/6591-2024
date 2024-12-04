// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;

        // Toggle for field relative driving
        public static boolean driveFieldRelative = true;


  }

  public static final class PneumaticsConstants {
    public static final int kOrangeUp = 4;
    public static final int kOrangeDown = 3; 
    public static final int kBlackUp = 5;
    public static final int kBlackDown = 2;
    public static final int kArmSolenoidUp = 7;
    public static final int kArmSolenoidDown = 0; 
    public static final double kCylinderDelay = 1000;
    
    //Pressure Transducer Stuff
    public static final int kPressureTransducerPort = 0;
    public static final double kScale = 250;
    public static final double kOffset = -25;

    
  }


  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class IntakeConstants {

    public static final int kIntakeFrontCANId = 9;
    public static final int kIntakeRearCANId = 11;
    public static final int kIntakeLeftCANId = 12;
    public static final int kIntakeRightCANId = 10;
    public static final int kGripperCANId = 13;

    public static final double kIntakeSpeed = 0.40;
    public static final double kEjectSpeed = 0.750;

    public static final IdleMode kIntakeMode = IdleMode.kCoast;
    public static final boolean kIntakeIsReversed = true;

    public static final IdleMode kGripperMode = IdleMode.kBrake;
    public static final boolean kGripperIsReversed = false;

    public static final double kGripperP =10;
    public static final double kGripperI = 0.0;
    public static final double kGripperD = 0.0; 
    public static final double kGripperFF = 15;  // Consider adding Feed-Forward to help hold the notes in place
    public static final double kGripperMinOutput = -1.0;
    public static final double kGripperMaxOutput = 1.0;

    public static final double kGripperEncoderPositionFactor = 360.0;  // 360 deg
    public static final double kGripperEncoderVelocityFactor = 60.0;  //  deg/sec

    public static final double kFingersOutAngle = 180; //Angle for the rotating plate
    public static final double kFingersInAngle = 90;  //Angle for the rotating plate
    public static final double kFingerDelayTimer = 3000; 

  }
   
  public static final class ShooterConstants {
    public static final int kShooterCANID = 15;
    public static final int kSliderCANID = 14;

    public static final double kNearShotSpeed = 0.7;
    public static final double kFarShotSpeed = 1.0;
    public static final double kBloopSpeed = 0.50;
    public static final double kintakeShooterSpeed = -0.5;

    public static final IdleMode kShooterMode = IdleMode.kCoast;
    public static final IdleMode kSliderMode = IdleMode.kBrake;
    public static final boolean kShooterIsReversed = false;
    public static final boolean kSliderIsReversed = false;
    
    public static final double kSliderP =0.70;
    public static final double kSliderI = 2.0e-5;
    public static final double kSliderD = 6.0e0-3; 
    public static final double kSliderFF = 0.0;
    public static final double kSliderMinOutput = -0.12;
    public static final double kSliderMaxOutput = 0.12;
 
    public static final double kSliderEncoderPositionFactor = 360.0;  // 360 deg
    public static final double kSliderEncoderVelocityFactor = 60.0;  //  Sdeg/sec

    public static final double kSliderShootPsn = 0; //position for slider
    public static final double kSliderParkPsn = -7000;  //position for slider
    public static final double kSliderLoadPsn = -32000; // position for loading slider
    //public static final double kFingerDelayTimer = 3000; 

    /**
     * Note: Any example colors should be calibrated as the user needs, these
     * are here as a basic example.
     */
    public static final Color kBlueTarget = new Color(0.157, 0.358, 0.484);
    public static final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public static final Color kRedTarget = new Color(0.463, 0.377, 0.159);
    public static final Color kYellowTarget = new Color(0.361, 0.524, 0.113);



  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
    
    // Controller Ports
    public static final int kLeftControllerPort = 0;
    public static final int kRightControllerPort = 1;
    public static final int kButtonBoardPort = 2 ;


    // Left Controller Buttons
    public static final int kSwitchCameraButton =1; //Left Trigger to switch between two cameras
    public static final int kSetXButton = 4; // Need to decide which stick and button we should use...
    public static final int kGyroRestButton = 5;  // Need to decide which stick and button we should use...
    public static final int kdriveDebugDataButton = 8; // maybe move to button board?





    // Right Controller Buttons
    public static final int kIntakeInButton = 1;
    public static final int kEjectButton = 2;
    public static final int kFieldRelativeButton = 3;

    // Button Board Buttons
    public static final int kArmUpButton = 1;
    public static final int kFingersInButton = 2;
    public static final int kFingerResetButton = 3;
    public static final int kOrangeUpButton = 4;
    public static final int kBlackUpButton = 5;
    public static final int kArmDownButton = 6;
    public static final int kFingersOutButton = 7;
    public static final int kGrabButton = 8;
    public static final int kOrangeDownButton = 9;
    public static final int kBlackDownButton = 10;
    public static final int kLoadButton = 11;
    public static final int kParkButton = 12;
    public static final int kSliderResetButton = 13;
    //public static final int Unused = 14:
    public static final int kIntakeShooterButton = 15;
    //public static final int Unused = 16:
    public static final int kFarShotButton = 17;
    public static final int kbloopButton = 18;
    public static final int kStopIntakeButton = 19; 
    public static final int kStopShooterButton = 20;
    
    
    
    



  }


}
