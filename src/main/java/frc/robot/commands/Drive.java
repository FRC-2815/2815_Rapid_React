// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain mDriveTrain = DriveTrain.getInstance();
  private DoubleSupplier mForwardAxis;
  private DoubleSupplier mTurnAxis;

  public Drive(DoubleSupplier f_Axis, DoubleSupplier t_axis) {
    mForwardAxis = f_Axis;
    mTurnAxis = t_axis;

    addRequirements(mDriveTrain);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    mDriveTrain.resetEncoders();
    mDriveTrain.resetGyro();
=======
    driveTrain.resetEncoders();
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    mDriveTrain.drive(mForwardAxis.getAsDouble(), mTurnAxis.getAsDouble());
    mDriveTrain.update();
=======
    driveTrain.drive(forwardAxis.getAsDouble(), turnAxis.getAsDouble());
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
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