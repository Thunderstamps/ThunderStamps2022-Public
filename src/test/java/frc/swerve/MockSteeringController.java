package frc.swerve;

import frc.robot.subsystems.swerve.ISteeringController;

public class MockSteeringController implements ISteeringController {
    private boolean homed = true;

    public void setHomed(boolean homed) {
        this.homed = homed;
    }
    
    @Override
    public boolean isHomed() {
        return this.homed;
    }

    private double targetMotorRev = 0.0;

    @Override
    public void setTargetMotorRev(double targetMotorRev) {
        this.targetMotorRev = targetMotorRev;
    }

    public double getTargetMotorRev() {
        return this.targetMotorRev;
    }

    private double motorRevCount = 0.0;

    public void setMotorRevCount(double motorRevCount) {
        this.motorRevCount = motorRevCount;
    }

    @Override
    public double getMotorRevCount() {
        return this.motorRevCount;
    }

    private double motorRevsPerSteeringRev = 9.0;

    public void setMotorRevsPerSteeringRev(
        double motorRevsPerSteeringRev) {
            this.motorRevsPerSteeringRev = motorRevsPerSteeringRev;
        }

    @Override
    public double getMotorRevsPerSteeringRev() {
        return this.motorRevsPerSteeringRev;
    }
    

}
