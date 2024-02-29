/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

  

/**
 * Add your docs here.
 */
public class PneumaticSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM); 
 
  /*
    //DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public DigitalInput elevForwardLimitSwitch = new DigitalInput(4);
    public DigitalInput elevReverseLimitSwitch = new DigitalInput(5);
    public DigitalInput mainPlateDownLimitSwitch = new DigitalInput(1);
    public DigitalInput mainPlateUpLimitSwitch = new DigitalInput(0);
    public DigitalInput backPlateDownLimitSwitch = new DigitalInput(3);
    public DigitalInput backPlateUpLimitSwitch = new DigitalInput(2);
  */

  // Joel's comments:  updated wiring on practace 'bot to be the same as 'real' 'bot
    public DoubleSolenoid m_armSolenoid =   new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.kArmSolenoidDown, PneumaticsConstants.kArmSolenoidUp);



  // Joel's comment:  Adding this section is what allows us to call the PneumaticSubsystem from other parts of the code
      public PneumaticSubsystem () {

      /*
      Joel's comment - These lines simply return the state of the compressor, pressure switch and compressor current
      These are only needed if we want to display them on the driverstation/smart dashboard
      If uncommented they show as unused variables since they aren't currently used anywhere else
      By having them in this section, they should be able to be called from other areas of the code (but I could be wrong aobut that!)
      // Get compressor current draw.
      return m_compressor.getCurrent();
      // Get whether the compressor is active.
      return m_compressor.isEnabled();
      // Get the digital pressure switch connected to the PCM/PH.
      // The switch is open when the pressure is over ~120 PSI.
      return m_compressor.getPressureSwitchValue();
      */
      }

  @Override

  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setArmUp () {
    m_armSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setArmDown (){
    m_armSolenoid.set(DoubleSolenoid.Value.kReverse
    );
  }

  
    // External analog pressure sensor
    // product-specific voltage->pressure conversion, see product manual
    // in this case, 250(V/5)-25
    // the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
    // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25

  public final AnalogPotentiometer m_pressureTransducer =
      new AnalogPotentiometer(PneumaticsConstants.kPressureTransducerPort, PneumaticsConstants.kScale, PneumaticsConstants.kOffset);

          // Get the pressure (in PSI) from an analog pressure sensor connected to the RIO.
          //return m_pressureTransducer.get();


}


  




