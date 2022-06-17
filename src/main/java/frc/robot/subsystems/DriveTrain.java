package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
    private final DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
                                                                                                KitbotMotor.kDoubleNEOPerSide,
                                                                                                KitbotGearing.k10p71,
                                                                                                KitbotWheelSize.kSixInch,
                                                                                                null
    );

    private final RelativeEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();    
    private final RelativeEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

    private final Gyro m_gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim((ADXRS450_Gyro)m_gyro);

    private final DifferentialDriveOdometry m_odometry;

    private final Field2d m_field = new Field2d();

    public DriveTrain() {
        m_leftFrontMotor.restoreFactoryDefaults();
        m_leftBackMotor.restoreFactoryDefaults();
        m_rightFrontMotor.restoreFactoryDefaults();
        m_rightBackMotor.restoreFactoryDefaults();

        m_leftBackMotor.follow(m_leftFrontMotor);
        m_rightBackMotor.follow(m_rightFrontMotor);   

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

        REVPhysicsSim.getInstance().addSparkMax(m_leftFrontMotor, DCMotor.getNEO(2));
        REVPhysicsSim.getInstance().addSparkMax(m_rightFrontMotor, DCMotor.getNEO(2));

        SmartDashboard.putData("field", m_field);
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInputs(m_leftFrontMotor.get() * RobotController.getInputVoltage(), m_rightFrontMotor.get() * RobotController.getInputVoltage());

        m_driveSim.update(0.02);

        REVPhysicsSim.getInstance().run();

        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                          -m_rightEncoder.getPosition());

                          var translation = m_odometry.getPoseMeters().getTranslation();
                          m_xEntry.setNumber(translation.getX());
                          m_yEntry.setNumber(translation.getY());
        SmartDashboard.putString("Pose", getPose().toString());
        SmartDashboard.putString("Pose Rot", getPose().getRotation().toString());
        SmartDashboard.putString("Pose Dist", getPose().getTranslation().toString());
        m_field.setRobotPose(m_odometry.getPoseMeters());
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
        String debug = debug(leftVolts, rightVolts);
        System.out.println(debug);
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

    public String debug(double l, double r) {
        SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
        SmartDashboard.putNumber("OdoGyro", getHeading());
        SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Position", -m_rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", -m_rightEncoder.getVelocity());
        SmartDashboard.putNumber("Left Voltage", l);
        SmartDashboard.putNumber("Right Voltage", r);
        return String.format("Gyro(%f); OdoGyro(%f); Position(%f, %f); Velocity(%f, %f); Voltage(%f, %f);", 
            m_gyro.getAngle(), getHeading(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(),
            m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity(), l, r
        );
    }
}