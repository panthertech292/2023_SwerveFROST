// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateManual extends CommandBase {
  private final ArmSubsystem ArmSub;
  private double v_armSpeed;
  private DoubleSupplier v_supArmSpeed;
  /** Creates a new ArmRotateManual. */
  public ArmRotateManual(ArmSubsystem s_ArmSubsystem, DoubleSupplier armSpeed) {
    ArmSub = s_ArmSubsystem;
    v_supArmSpeed = armSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (v_supArmSpeed.getAsDouble() > 0.10 || v_supArmSpeed.getAsDouble() < -0.10){ //Deadband TODO: Implement this somewhere else.
      this.v_armSpeed = v_supArmSpeed.getAsDouble();
    }else{
      this.v_armSpeed = 0;
    }
    SmartDashboard.putNumber("Arm Speed", v_armSpeed);
    ArmSub.setArmRotateMotor(v_armSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotateMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
