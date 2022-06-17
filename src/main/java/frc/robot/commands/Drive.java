// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain driveTrain;
  private final DoubleSupplier forwardAxis;
  private final DoubleSupplier turnAxis;

  public Drive(DriveTrain DT, DoubleSupplier f_Axis, DoubleSupplier t_axis) {
    driveTrain = DT;
    forwardAxis = f_Axis;
    turnAxis = t_axis;

    addRequirements(driveTrain);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(forwardAxis.getAsDouble(), turnAxis.getAsDouble());
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