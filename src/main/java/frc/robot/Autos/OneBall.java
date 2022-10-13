package frc.robot.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDump;
import frc.robot.commands.AutoCommands.DriveDistance;

public class OneBall extends SequentialCommandGroup {
    
    public OneBall() {
        addCommands(new AutoDump(3));
        addCommands(new DriveDistance(Units.inchesToMeters(100)));
    }
}
