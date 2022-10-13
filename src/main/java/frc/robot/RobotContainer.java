// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.Autos.OneBall;
import frc.robot.Autos.TwoBall;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Index;
import frc.robot.commands.Lift;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Lifter;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain mDriveTrain = DriveTrain.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Climber mClimber = Climber.getInstance();
  private final Lifter mLifter = Lifter.getInstance();

  private Drive mDrive;
  private Index mIndex;
  private Climb mClimb;
  private Lift mLift;

  SendableChooser<CommandGroupBase> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();

    autoChooser.addOption("OneBall", new OneBall());
    autoChooser.addOption("TwoBall", new TwoBall());

    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    XboxController mController = new XboxController(0);
    Joystick mJoystick = new Joystick(1);

    mDrive = new Drive(() -> mController.getRightX(), () -> mController.getLeftY());
    mIndex = new Index(() -> mJoystick.getRawButton(2), () -> mJoystick.getRawButton(1));
    mClimb = new Climb(() -> mController.getLeftBumper(), () -> mController.getRightBumper());
    mLift = new Lift(() -> mJoystick.getRawButton(3), () -> mJoystick.getRawButton(4));

    mDriveTrain.setDefaultCommand(mDrive);
    mIndexer.setDefaultCommand(mIndex);
    mClimber.setDefaultCommand(mClimb);
    mLifter.setDefaultCommand(mLift);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}