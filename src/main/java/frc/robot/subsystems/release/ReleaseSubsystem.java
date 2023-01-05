package frc.robot.subsystems.release;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj2.command.*;

public class ReleaseSubsystem extends SubsystemBase {
    
    private final static int CURRENT_LIMIT_A = 20;
    private final CANSparkMax smax = new CANSparkMax(34, MotorType.kBrushless);
    
    private boolean fullCurrent = false;

    public ReleaseSubsystem() {
        smax.setInverted(false); // never invert a SparkMAX
        smax.setIdleMode(IdleMode.kBrake);
        smax.setSmartCurrentLimit(CURRENT_LIMIT_A); // 20 A limit allows continuous locked rotor current
        //smax.burnFlash();
    }

    public void executeLock() {
        if(fullCurrent) {
            smax.setSmartCurrentLimit(CURRENT_LIMIT_A);
            this.fullCurrent = false;
        }
        smax.set(1);
    }

    public void executeRelease() {
        if(!fullCurrent) {
            smax.setSmartCurrentLimit(80);
            this.fullCurrent = true;
        }
        smax.set(-1);
    }

    public void executeStop() {
        smax.set(0);
    }
}
