package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {
    private final Indexer mIndexer = Indexer.getInstance();
    private BooleanSupplier mIntake;
    private BooleanSupplier mExhaust;
    private boolean mFinished;

    public Index(BooleanSupplier i, BooleanSupplier e) {

        addRequirements(mIndexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (mIntake.getAsBoolean()) {
            mIndexer.intake();
        } else if (mExhaust.getAsBoolean()) {
            mIndexer.exhaust();
        } else {
            mIndexer.off();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }

}
