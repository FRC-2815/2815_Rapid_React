package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AngleCorrect;
import frc.robot.commands.AutoCommands.AutoDump;
import frc.robot.commands.AutoCommands.AutoLower;
import frc.robot.commands.AutoCommands.DriveDistance;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Lifter;

public class TwoBall extends SequentialCommandGroup {
    private final Indexer mIndexer = Indexer.getInstance();
    private final Lifter mLifter = Lifter.getInstance();
    
    public TwoBall() {
        addCommands(new AutoLower(3));
        addCommands(new DriveDistance(56).raceWith(new RunCommand(() -> mIndexer.intake(), mIndexer)));
        addCommands(new InstantCommand(() -> mIndexer.off()));
        addCommands(new AngleCorrect(135));
        addCommands(new DriveDistance(100).alongWith(new RunCommand(() -> mLifter.lift(), mLifter)));
        addCommands(new AngleCorrect(90));
        addCommands(new DriveDistance(60));
        addCommands(new AutoDump(3));
    }
}
