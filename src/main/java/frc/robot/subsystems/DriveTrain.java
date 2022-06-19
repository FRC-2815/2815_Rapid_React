package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    private MotorControllerGroup m_Left = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    private MotorControllerGroup m_Right = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

    private DifferentialDrive m_drive = new DifferentialDrive(m_Left, m_Right);

    private RelativeEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();    
    private RelativeEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

    private SimDeviceSim leftSparkSim = new SimDeviceSim("SPARK MAX [1]");
    private SimDeviceSim rightSparkSim = new SimDeviceSim("SPARK MAX [2]");

    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private DifferentialDriveOdometry m_odometry;

    private ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(m_gyro);

    private final DifferentialDrivetrainSim m_DriveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null
    );

    private Field2d field = new Field2d();

    public DriveTrain() {
        m_leftFrontMotor.restoreFactoryDefaults();
        m_leftBackMotor.restoreFactoryDefaults();
        m_rightFrontMotor.restoreFactoryDefaults();
        m_rightBackMotor.restoreFactoryDefaults();

        m_leftEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
        m_rightEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
        m_leftEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
        m_rightEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
        
        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        m_gyro.reset();

        m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
        m_leftBackMotor.setIdleMode(IdleMode.kBrake);
        m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
        m_rightBackMotor.setIdleMode(IdleMode.kBrake);

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getPosition(),
                    m_rightEncoder.getPosition());
        field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("leftMotor.get(): " + m_leftFrontMotor.get() + "    rightMotor.get(): " + m_rightFrontMotor.get());
        m_DriveSim.setInputs(
            m_leftFrontMotor.get() * RobotController.getInputVoltage(),
            m_rightFrontMotor.get() * RobotController.getInputVoltage()
        );

        m_DriveSim.update(0.02);

        leftSparkSim.getDouble("Position").set(m_DriveSim.getLeftPositionMeters());
        leftSparkSim.getDouble("Velocity").set(m_DriveSim.getLeftVelocityMetersPerSecond());
        rightSparkSim.getDouble("Position").set(m_DriveSim.getLeftPositionMeters());
        rightSparkSim.getDouble("Velocity").set(m_DriveSim.getLeftVelocityMetersPerSecond());
        gyroSim.setAngle(-m_DriveSim.getHeading().getDegrees());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void drive(double forward, double turn) {
        m_drive.arcadeDrive(forward, turn);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftFrontMotor.setVoltage(leftVolts);
        m_rightFrontMotor.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + -m_rightEncoder.getPosition()) / 2.0;
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

}