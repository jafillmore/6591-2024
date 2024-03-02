// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;



public class Shootersubsystem extends SubsystemBase {
  /** Creates a new Shootersubsystem. */
  final CANSparkMax m_ShooterSparkMax = new CANSparkMax(ShooterConstants.kShooterCANID, MotorType.kBrushless);
  final CANSparkMax m_SliderSparkMax = new CANSparkMax(ShooterConstants.kSliderCANID, MotorType.kBrushless);
       
  private final SparkPIDController m_sliderPIDController;  
  private final RelativeEncoder m_sliderEncoder;


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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
  public void setSlider (double sliderPosition) {
    m_sliderPIDController.setReference(sliderPosition, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber(   "Slider Position", sliderPosition);
  };

  public void shoot(double shooterPower, double sliderPosition) {
    m_ShooterSparkMax.set(shooterPower);
    m_sliderPIDController.setReference(sliderPosition, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber(   "Slider Position", sliderPosition);
    zeroSlider();
    Timer.delay(2);
    m_sliderPIDController.setReference(ShooterConstants.kSliderParkPsn, CANSparkMax.ControlType.kPosition);
  };

  public void zeroSlider(){
    m_SliderSparkMax.set(0.15);
    Timer.delay(4);
    m_SliderSparkMax.set(0.0);
    m_sliderEncoder.setPosition(0);
  }
  
  public void shooterStop(){
    m_SliderSparkMax.set(0.0);
    m_ShooterSparkMax.set(0.0);
  }

}
