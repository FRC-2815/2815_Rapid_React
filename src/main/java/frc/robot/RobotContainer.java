// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.Joystick;
=======
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.Autos.OneBall;
import frc.robot.Autos.TwoBall;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Index;
import frc.robot.commands.Lift;
import frc.robot.subsystems.Climber;
=======
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
<<<<<<< HEAD
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
=======
public class RobotContainer {

  private final DriveTrain m_driveTrain = new DriveTrain();
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361

  private Drive drive;

  public RobotContainer() {
<<<<<<< HEAD
    autoChooser = new SendableChooser<>();

    autoChooser.addOption("OneBall", new OneBall());
    autoChooser.addOption("TwoBall", new TwoBall());

    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
=======
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
    configureButtonBindings();
  }

  private void configureButtonBindings() {
<<<<<<< HEAD
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
=======
    XboxController controller = new XboxController(0);
    
    drive = new Drive(m_driveTrain, () -> controller.getLeftY(), () -> controller.getRightX());

    m_driveTrain.setDefaultCommand(drive);
>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
  }

  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(3, 3, new Rotation2d(0)),
            List.of(new Translation2d(6, 0)),
            new Pose2d(6, 3, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_driveTrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,
            m_driveTrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            m_driveTrain::tankDriveVolts,
            m_driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
  }
}