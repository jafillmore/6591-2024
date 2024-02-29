// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {
    final CANSparkMax m_intakeSparkMaxFront = new CANSparkMax(IntakeConstants.kIntakeFrontCANId, MotorType.kBrushless);
    final CANSparkMax m_intakeSparkMaxRear = new CANSparkMax(IntakeConstants.kIntakeRearCANId, MotorType.kBrushless);
    final CANSparkMax m_intakeSparkMaxLeft = new CANSparkMax(IntakeConstants.kIntakeLeftCANId, MotorType.kBrushless);
    final CANSparkMax m_intakeSparkMaxRight = new CANSparkMax(IntakeConstants.kIntakeRightCANId, MotorType.kBrushless);
    final CANSparkMax m_gripperSparkMax = new CANSparkMax(IntakeConstants.kGripperCANId, MotorType.kBrushed);
  
    private final AbsoluteEncoder m_gripperEncoder;

    private final SparkPIDController m_gripperPIDController;
    private double newOffSet = 0.0;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeSparkMaxFront.setIdleMode(IntakeConstants.kIntakeMode);
    m_intakeSparkMaxRear.setIdleMode(IntakeConstants.kIntakeMode);
    m_intakeSparkMaxLeft.setIdleMode(IntakeConstants.kIntakeMode);
    m_intakeSparkMaxRight.setIdleMode(IntakeConstants.kIntakeMode);
    m_gripperSparkMax.setIdleMode(IntakeConstants.kGripperMode);
    
    m_intakeSparkMaxFront.setInverted(IntakeConstants.kIntakeIsReversed);
    m_intakeSparkMaxRear.setInverted(IntakeConstants.kIntakeIsReversed);
    m_intakeSparkMaxLeft.setInverted(IntakeConstants.kIntakeIsReversed);
    m_intakeSparkMaxRight.setInverted(IntakeConstants.kIntakeIsReversed);
    m_gripperSparkMax.setInverted(IntakeConstants.kGripperIsReversed);

    // Setup encoders and PID controllers for the claws.
    m_gripperEncoder = m_gripperSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_gripperEncoder.setInverted(false);
    m_gripperPIDController = m_gripperSparkMax.getPIDController();
    m_gripperPIDController.setFeedbackDevice(m_gripperEncoder);
    
    // Enable PID wrap around for the gripper motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_gripperPIDController.setPositionPIDWrappingEnabled(false);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_gripperEncoder.setPositionConversionFactor(IntakeConstants.kGripperEncoderPositionFactor);
    m_gripperEncoder.setVelocityConversionFactor(IntakeConstants.kGripperEncoderVelocityFactor);
  
    // Set the PID gains for the turning motor. Note these are a wild guess! may need to tune!
    m_gripperPIDController.setP(IntakeConstants.kGripperP);
    m_gripperPIDController.setI(IntakeConstants.kGripperI);
    m_gripperPIDController.setD(IntakeConstants.kGripperD);
    m_gripperPIDController.setFF(IntakeConstants.kGripperFF);
    m_gripperPIDController.setOutputRange(IntakeConstants.kGripperMinOutput, 
        IntakeConstants.kGripperMaxOutput);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_intakeSparkMaxFront.burnFlash();
    m_intakeSparkMaxRear.burnFlash();
    m_intakeSparkMaxLeft.burnFlash();
    m_intakeSparkMaxRight.burnFlash();
    m_gripperSparkMax.burnFlash();
    
       
  }
  
    
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber(   "Gripper Actual Angle", m_gripperEncoder.getPosition());
    //SmartDashboard.putNumber(   "Gripper Zero Offset", m_gripperEncoder.getZeroOffset());
  }

  public void setIntake (double intakePower) {
        m_intakeSparkMaxFront.set(intakePower);
        m_intakeSparkMaxRear.set(intakePower);
        m_intakeSparkMaxLeft.set(intakePower);
        m_intakeSparkMaxRight.set(intakePower);

      };

  public void setGripper (double gripperAngle) {
        m_gripperPIDController.setReference(gripperAngle, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber(   "Gripper Target Angle", gripperAngle);
        
  }
  public void zeroFingers(){
    m_gripperEncoder.setZeroOffset(0.0);
    m_gripperSparkMax.set(0.75);
    Timer.delay(.5);
    m_gripperSparkMax.set(-0.75);
    Timer.delay(1);
    newOffSet = m_gripperEncoder.getPosition();
    m_gripperEncoder.setZeroOffset(newOffSet);
    m_gripperSparkMax.set(0);
    m_gripperSparkMax.burnFlash();
  }
  /*
  public void grab(){
        m_gripperPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void drop(){
        m_gripperPIDController.setReference(89, CANSparkMax.ControlType.kPosition);
  }
  */
}

