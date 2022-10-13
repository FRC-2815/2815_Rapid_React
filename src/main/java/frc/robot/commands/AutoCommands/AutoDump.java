package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class AutoDump extends CommandBase {
    private final Indexer mIndexer = Indexer.getInstance();
    private double mSeconds;
    private boolean finished;

    private final Timer timer;
    
    public AutoDump(double s) {
        mSeconds = s;
        timer = new Timer();

        addRequirements(mIndexer);
    }

    @Override
    public void initialize() {
        timer.start();
        mIndexer.exhaust();
    }

    @Override
    public void execute() {
        if (timer.get() >= mSeconds) {
            finished = true;
        }
        mIndexer.exhaust();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}