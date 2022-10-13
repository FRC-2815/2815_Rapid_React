package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AngleCorrect extends CommandBase {
    private final DriveTrain mDriveTrain = DriveTrain.getInstance();
    private double mAngle;
    private boolean finished;

    public AngleCorrect(double a) {
        mAngle = a;

        addRequirements(mDriveTrain);
    }

    @Override
    public void initialize() {
        reset();
        mDriveTrain.setAngle(mAngle);
    }

    @Override
    public void execute() {
        finished = mDriveTrain.rotatePID();
        mDriveTrain.update();
    }
    @Override
    public void end(boolean interrupted) {
        finished = false;
        reset();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    private void reset() {
        mDriveTrain.resetGyro();
        mDriveTrain.resetEncoders();
        mDriveTrain.setCurrentHedding();
    }
}