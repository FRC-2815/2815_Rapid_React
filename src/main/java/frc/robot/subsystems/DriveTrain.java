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
<<<<<<< HEAD
    private static DriveTrain mInstance;

    private CANSparkMax mLeftLeader;
    private CANSparkMax mRightLeader;
    
    private CANSparkMax mLeftFollower;
    private CANSparkMax mRightFollower;

    private RelativeEncoder mLeftEncoder;
    private RelativeEncoder mRightEncoder;

    private SparkMaxPIDController mLeftController;
    private SparkMaxPIDController mRightController;

    private DifferentialDrive mDrive;

    private ADXRS450_Gyro mGyro;
    private SimpleMotorFeedforward mFeedforward;
    private PIDController mGyroController;
    
    private double mSetpoint;
    private double mAngle;
    private double mLeftEncoderPosition;
    private double mRightEncoderPosition;
    private double mDesiredAngle;

    private double mkP;
    private double mkI;
    private double mkD;

    public DriveTrain() {
        mLeftLeader = new CANSparkMax(1, MotorType.kBrushless);
        mRightLeader = new CANSparkMax(3, MotorType.kBrushless);

        mLeftFollower = new CANSparkMax(2, MotorType.kBrushless);
        mRightFollower = new CANSparkMax(4, MotorType.kBrushless);

        mLeftEncoder = mLeftLeader.getEncoder();
        mRightEncoder = mRightLeader.getEncoder();

        mLeftController = mLeftLeader.getPIDController();
        mRightController = mRightLeader.getPIDController();

        mDrive = new DifferentialDrive(mLeftLeader, mRightLeader);

        mGyro = new ADXRS450_Gyro();
        mGyro.calibrate();
        mGyroController = new PIDController(0, 0, 0);
        mFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

        mLeftFollower.follow(mLeftLeader);
        mRightFollower.follow(mRightLeader);
        mLeftController.setP(Constants.kP);
        mLeftController.setI(Constants.kI);
        mLeftController.setD(Constants.kD);
        mRightController.setP(Constants.kP);
        mRightController.setI(Constants.kI);
        mRightController.setD(Constants.kD);

        mLeftController.setOutputRange(-0.5, 0.5);
        mRightController.setOutputRange(-0.5, 0.5);

        mDrive.setSafetyEnabled(false);
    }

    public static DriveTrain getInstance() {
        if (mInstance == null) {
            mInstance = new DriveTrain();
        }
        return mInstance;
    }

    public void resetEncoders() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    public void drive(double f, double t) {
        // drive.arcadeDrive(t, f);
        mDrive.arcadeDrive(f, t);
        SmartDashboard.putNumber("leftEncoder", mLeftEncoder.getPosition());
        SmartDashboard.putNumber("rightEncoder", mRightEncoder.getPosition());
    }

    public void setSetpoint(double s) {
        mSetpoint = s * (Constants.GEAR_BOX_RATIO / Constants.WHEEL_CIRCUMFERENCE) * Constants.ELLIOT_COEFFICIENT;  // MAKE SURE TO CONVERT FROM METERS TO ROTATIONS BEFORE MULTIPLYING BY THIS COEFFICIENT 1 ROTATION = THE COEFFICIENT
    }

    public void drivePID() {
        mLeftController.setReference(mSetpoint, CANSparkMax.ControlType.kPosition);
        mRightController.setReference(-mSetpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("leftEncoder", Math.abs(mLeftEncoder.getPosition()));
        SmartDashboard.putNumber("rightEncoder", Math.abs(mRightEncoder.getPosition()));
    }

    public boolean rotatePID() {
        mkP = SmartDashboard.getNumber("Rotational P", Constants.r_kP);
        mkI = SmartDashboard.getNumber("Rotational I", Constants.r_kI);
        mkD = SmartDashboard.getNumber("Rotational D", Constants.r_kD);
        double currentAngle = mGyro.getAngle();
        double finalAngle = mAngle + mDesiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", currentAngle);
        SmartDashboard.putNumber("Rotation Rate", mGyro.getRate());

        mGyroController.setPID(mkP, mkI, mkD);

        if (finalAngle < 0) {
            if (currentAngle <= finalAngle) {
                mDrive.arcadeDrive(0, 0);
                return true;
            }
            mDrive.arcadeDrive(mGyroController.calculate(currentAngle, finalAngle) + mFeedforward.calculate(Math.PI, 2), 0);
            // drive.arcadeDrive(0, -gyroController.calculate(currentAngle, finalAngle));
        }

        if (finalAngle > 0) {
            if (currentAngle >= finalAngle) {
                mDrive.arcadeDrive(0, 0);
                return true; 
            }
            mDrive.arcadeDrive(mGyroController.calculate(currentAngle, finalAngle) + mFeedforward.calculate(Math.PI, 2), 0);
            // drive.arcadeDrive(0, gyroController.calculate(currentAngle, finalAngle));
        }
        return false;
    }

    public boolean rotate() {
        double currentAngle = mGyro.getAngle();
        double finalAngle = mAngle + mDesiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", mGyro.getAngle());
        if (Math.abs(currentAngle) >= Math.abs(finalAngle)) {
            mDrive.arcadeDrive(0, 0);
            return true;
        }

        if (finalAngle < 0) {
            mDrive.arcadeDrive(-0.5, 0);
        }

        if (finalAngle > 0) {
            mDrive.arcadeDrive(0.5, 0);
        }
        return false;
    }

    public boolean rotateSlow() {
        double currentAngle = mGyro.getAngle();
        double finalAngle = mAngle + mDesiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", mGyro.getAngle());
        if (Math.abs(currentAngle) >= Math.abs(finalAngle)) {
            mDrive.arcadeDrive(0, 0);
            return true;
        }

        if (finalAngle < 0) {
            mDrive.arcadeDrive(-0.25, 0);
        }

        if (finalAngle > 0) {
            mDrive.arcadeDrive(0.25, 0);
        }
        return false;
    }

    public void resetGyro() {
        mGyro.reset();
    }

    public boolean leftAtSetpoint() {
        if ((mLeftEncoderPosition == Math.abs(mSetpoint + 2)) || (mLeftEncoderPosition == Math.abs(mSetpoint - 2))) {
            return true;
        }
        return false;
    }

    public boolean rightAtSetpoint() {
        if ((mRightEncoderPosition == Math.abs(mSetpoint + 2)) || (mRightEncoderPosition == Math.abs(mSetpoint - 2))) {
            return true;
        }
        return false;
    }

    public double getleftVelocity() {
        return mLeftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return mRightEncoder.getVelocity();
    }

    public double getVelocity() {
        return (getleftVelocity() + getRightVelocity()) / 2;
    }

    public boolean atSetpoint() {
        if (Math.abs(mSetpoint) <= Math.abs(mLeftEncoder.getPosition())) {
            System.out.print("\n\nyay\n\n");
            return true;
        }
        return false;
    }

    public void update() {
        mLeftEncoderPosition = Math.abs(mLeftEncoder.getPosition());
        mRightEncoderPosition = Math.abs(mRightEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    public void setAngle(double a) {
        mDesiredAngle = a;
    }

    public void setCurrentHedding() {
        mAngle = mGyro.getAngle();
    }

    public void setRotationalPID(double kP, double kI, double kD) {
        mGyroController.setPID(kP, kI, kD);
    }
=======
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

>>>>>>> cebed2c35452ba9ca70f570a80ee54390eb27361
}