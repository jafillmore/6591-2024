// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;




public class Shootersubsystem extends SubsystemBase {
  /** Creates a new Shootersubsystem. */
  final CANSparkMax m_ShooterSparkMax = new CANSparkMax(ShooterConstants.kShooterCANID, MotorType.kBrushless);
  final CANSparkMax m_SliderSparkMax = new CANSparkMax(ShooterConstants.kSliderCANID, MotorType.kBrushless);
       
  private final SparkPIDController m_sliderPIDController;  
  private final RelativeEncoder m_sliderEncoder;

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private static Color detectedColor;
  private static String colorString;
  private static String positionString = "Don't Know";
  private static ColorMatchResult match;
  

  public Shootersubsystem() {
    m_ShooterSparkMax.setIdleMode(ShooterConstants.kShooterMode);
    m_SliderSparkMax.setIdleMode(ShooterConstants.kSliderMode);

    m_ShooterSparkMax.setInverted(ShooterConstants.kShooterIsReversed);
    m_SliderSparkMax.setInverted(ShooterConstants.kSliderIsReversed);

    // Setup encoders and PID controllers for the Slider SPARK MAX.
    m_sliderEncoder = m_SliderSparkMax.getEncoder();
    //m_sliderEncoder.setInverted(false);
    m_sliderPIDController = m_SliderSparkMax.getPIDController();
    m_sliderPIDController.setFeedbackDevice(m_sliderEncoder);
    

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_sliderEncoder.setPositionConversionFactor(ShooterConstants.kSliderEncoderPositionFactor);
    m_sliderEncoder.setVelocityConversionFactor(ShooterConstants.kSliderEncoderVelocityFactor);
  
    // Set the PID gains for the turning motor. Note these are a wild guess! may need to tune!
    m_sliderPIDController.setP(ShooterConstants.kSliderP);
    m_sliderPIDController.setI(ShooterConstants.kSliderI);
    m_sliderPIDController.setD(ShooterConstants.kSliderD);
    m_sliderPIDController.setFF(ShooterConstants.kSliderFF);
    m_sliderPIDController.setOutputRange(ShooterConstants.kSliderMinOutput, 
        ShooterConstants.kSliderMaxOutput);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_ShooterSparkMax.burnFlash();
    m_SliderSparkMax.burnFlash();

    m_colorMatcher.addColorMatch(Constants.ShooterConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.ShooterConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.ShooterConstants.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.ShooterConstants.kYellowTarget);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    bestmatch();
    



  }
  public void setSlider (double sliderPosition) {
    m_sliderPIDController.setReference(sliderPosition, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber(   "Slider Position", sliderPosition);
  };

  public void shoot(double shooterPower  /*, double sliderPosition*/) {
    m_ShooterSparkMax.set(shooterPower);
    //m_sliderPIDController.setReference(sliderPosition, CANSparkMax.ControlType.kPosition);
    //SmartDashboard.putNumber(   "Slider Position", sliderPosition);
    zeroSlider();
    //Timer.delay(10);
    //m_sliderPIDController.setReference(ShooterConstants.kSliderParkPsn, CANSparkMax.ControlType.kPosition);
  }
  
  public void zeroSlider(){
    /* 
    while (true) {   
      m_SliderSparkMax.set(0.05);
      positionString = "Going Up";
      bestmatch();
  
     
      if (match.color == Constants.ShooterConstants.kRedTarget) { 
        m_SliderSparkMax.set(0.0);
        m_sliderEncoder.setPosition(0);
        positionString = "At Zero";
        break;
      } 
    }
    */
    m_SliderSparkMax.set(0.15);

    Timer.delay(5);
       
    m_SliderSparkMax.set(0.0);
    m_sliderEncoder.setPosition(0);
  }
  
public void downSlider(){
    
    m_SliderSparkMax.set(-0.05);
    
    // Loop while enabled until color is matched
    while (RobotState.isEnabled()) {
      positionString = "Going Down";
      if (match.color == Constants.ShooterConstants.kBlueTarget) { 
        m_SliderSparkMax.set(0.0);
        colorString = "Is down";
        break;
      }
    }

    m_SliderSparkMax.set(0.0);

  }




  public void shooterStop(){
    m_SliderSparkMax.set(0.0);
    m_ShooterSparkMax.set(0.0);
  }

  public void bestmatch(){
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);


    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    
     if (match.color == Constants.ShooterConstants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.ShooterConstants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.ShooterConstants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.ShooterConstants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }


    
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putString("Position", positionString);
    SmartDashboard.putBoolean("Is Enabled", RobotState.isEnabled());
    SmartDashboard.putBoolean("Is Disabled", RobotState.isDisabled());
  }



}
