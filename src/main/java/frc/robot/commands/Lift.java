package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;

public class Lift extends CommandBase {

    private final Lifter mLifter = Lifter.getInstance();
    private BooleanSupplier mLift;
    private BooleanSupplier mLower;
    private boolean mFinished;

    public Lift(BooleanSupplier u, BooleanSupplier l) {
        mLift = u;
        mLower = l;

        addRequirements(mLifter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (mLift.getAsBoolean()) {
            mLifter.lift();
        } else if (mLower.getAsBoolean()) {
            mLifter.lower();
        } else {
            mLifter.off();
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
