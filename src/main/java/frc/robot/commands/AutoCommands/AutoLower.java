package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;

public class AutoLower extends CommandBase {
    private final Lifter mLifter = Lifter.getInstance();
    private boolean mFinished;
    private double mSeconds;

    public final Timer timer;
    
    public AutoLower(double s) {
        mSeconds = s;
        timer = new Timer();

        addRequirements(mLifter);
    }

    @Override
    public void initialize() {
        timer.start();
        mLifter.lower();
    }

    @Override
    public void execute() {
        if (timer.get() >= mSeconds) {
            mFinished = true;
        }
        mLifter.lower();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }
}