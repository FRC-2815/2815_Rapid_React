package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private final Climber mClimber = Climber.getInstance();
    private BooleanSupplier mLeftBumper;
    private BooleanSupplier mRightBumper;
    private boolean mFinished;

    public Climb(BooleanSupplier lB, BooleanSupplier rB) {
        mLeftBumper = lB;
        mRightBumper = rB;

        addRequirements(mClimber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (mLeftBumper.getAsBoolean()) {
            mClimber.up();
        } else if (mRightBumper.getAsBoolean()) {
            mClimber.down();
        } else {
            mClimber.off();
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
