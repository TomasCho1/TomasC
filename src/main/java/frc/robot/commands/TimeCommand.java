// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TimeCommand extends CommandBase {
  /** Creates a new TimeCommand. */
  DriveTrain tc; 
  Timer timer;
  public TimeCommand(DriveTrain TimeCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    tc = TimeCommand;
    addRequirements(tc);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= 3) {
      tc.tankDrive(0.8, 0.8);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 3){
      return true;
    }
    else {
      return false;
    }


  }
}
