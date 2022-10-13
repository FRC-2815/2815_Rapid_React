package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private static Indexer mInstance;

    private CANSparkMax mIndexer;

    public Indexer() {
        mIndexer = new CANSparkMax(5, MotorType.kBrushless);

        mIndexer.setIdleMode(IdleMode.kBrake);
    }

    public static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public void intake() {
        mIndexer.set(1);
    }

    public void exhaust() {
        mIndexer.set(-1);
    }
    
    public void off() {
        mIndexer.set(0);
    }
}