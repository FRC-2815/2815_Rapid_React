package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lifter extends SubsystemBase {
    private static Lifter mInstance;

    private CANSparkMax lifterLeader;
    private DigitalInput topLimit;

    public Lifter() {
        lifterLeader = new CANSparkMax(6, MotorType.kBrushless);
        lifterLeader.setIdleMode(IdleMode.kBrake);
        topLimit = new DigitalInput(0);
    }

    public static Lifter getInstance() {
        if (mInstance == null) {
            mInstance = new Lifter();
        }
        return mInstance;
    }

    public boolean lift() {
        if (!topLimit.get()) {
            lifterLeader.set(1);
            return false;
        }
        return true;
    }

    public void lower() {
        lifterLeader.set(-1);
    }

    public void off() {
        lifterLeader.set(0);
    }
}