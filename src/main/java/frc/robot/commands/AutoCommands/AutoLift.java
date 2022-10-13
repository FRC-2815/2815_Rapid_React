package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;

public class AutoLift extends CommandBase {
    private final Lifter mLifter = Lifter.getInstance();
    boolean mFinished;
    
    public AutoLift() {
        addRequirements(mLifter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mFinished = mLifter.lift();
    }

    @Override
    public void end(boolean interrupted) {
        mLifter.off();
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }
}