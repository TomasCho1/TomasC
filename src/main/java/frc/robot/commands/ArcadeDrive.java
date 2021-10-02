// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  DriveTrain driveTrain;
  Joystick lJoy;
  public ArcadeDrive(DriveTrain dt, Joystick lj) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    lJoy = lj;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -0.8 * lJoy.getRawAxis(Constants.JoystickAxis.YAxis);
    double zRotation = 0.8 * lJoy.getRawAxis(Constants.JoystickAxis.XAxis);

    driveTrain.arcadeDrive(xSpeed, zRotation);

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
