// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax ArmRotateMotor;
  private final DutyCycleEncoder ArmRotateEncoder;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    ArmRotateMotor = new CANSparkMax(20, MotorType.kBrushless); //TODO: Make constant
    ArmRotateMotor.restoreFactoryDefaults();
    ArmRotateMotor.setIdleMode(IdleMode.kBrake);
    ArmRotateMotor.burnFlash();

    ArmRotateEncoder = new DutyCycleEncoder(3); //TODO: Make constant & get value
  }

  public void setArmRotateMotor(double speed){
    ArmRotateMotor.set(speed);
  }
  public double getArmRotateEncoder(){
    return ArmRotateEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
