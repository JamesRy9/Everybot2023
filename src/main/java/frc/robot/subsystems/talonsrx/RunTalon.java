// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.talonsrx;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTalon extends CommandBase {
  private TalonSubsystem talonSubsystem;
  /** Creates a new RunTalonSubsystem. */
  public RunTalon(TalonSubsystem talonSubsystem) {
    this.talonSubsystem = talonSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(talonSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talonSubsystem.setPower(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
