package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
    private final DriveTrain mDriveTrain = DriveTrain.getInstance();
    private double mSetpoint;
    boolean mFinished;

    public DriveDistance(double s) {
        mSetpoint = s;

        addRequirements(mDriveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveTrain.resetGyro();
        mDriveTrain.resetEncoders();
        mDriveTrain.setSetpoint(mSetpoint);
        mDriveTrain.drivePID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveTrain.setAngle(0);
        if (mDriveTrain.atSetpoint()) {
            mFinished = true;
        } else {
            mDriveTrain.update();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveTrain.drive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (mFinished) {
            mDriveTrain.rotateSlow();
            mDriveTrain.resetEncoders();
            mDriveTrain.resetGyro();
        }
        return mFinished;
    }
}