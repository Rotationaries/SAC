// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
  private boolean isIntakeUp;
  public Joystick joystick = new Joystick(1);
  /** Creates a new Intake. */
  public Intake()  {   
    System.out.println("Intake Started");
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  public boolean isIntakeUp() {
    return isIntakeUp;
  }

  public void intakeDrive(){
    if (joystick.getRawButton(11)){
      System.out.println("11");
      intakeMotor.set(0);
    }
  
    if(joystick.getRawButton(1)){ //Pullin
      intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }
    else if (joystick.getRawButton(2)){ //Eject
      intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    }
    else {intakeMotor.set(0);}

    

}
}
  


