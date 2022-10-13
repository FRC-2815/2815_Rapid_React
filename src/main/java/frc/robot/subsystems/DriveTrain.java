package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
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
}