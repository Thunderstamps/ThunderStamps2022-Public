package frc.swerve;

import frc.robot.subsystems.swerve.IDriveController;

public class MockDriveController implements IDriveController {

    private double targetMotorRpm = 0.0;
    
    @Override
    public void executeVelocityMode(double targetMotorRpm){
        this.targetMotorRpm = targetMotorRpm;
    }

    public double getTargetMotorRpm() {
        return this.targetMotorRpm;
    }

    private double targetMotorRev = 0.0;

    @Override
    public void executePositionMode(double targetMotorRev) {
        this.targetMotorRev = targetMotorRev;
    }

    public double getTargetMotorRev() {
        return this.targetMotorRev;
    }

    private double motorRpm = 0.0;

    public void setMotorRpm(double motorRpm) {
        this.motorRpm = motorRpm;
    }
    
    @Override
    public double getMotorRpm() {
        return this.motorRpm;
    }

    private double motorRevCount = 0.0;

    public void setMotorRevCount(double motorRevCount) {
        this.motorRevCount = motorRevCount;
    }

    @Override
    public double getMotorRevCount() {
        return this.motorRevCount;
    }

    private double motorRevsPerWheelRev = 8.0;

    public void setMotorRevsPerWheelRev(double motorRevsPerWheelRev) {
        this.motorRevsPerWheelRev = motorRevsPerWheelRev;
    }

    @Override
    public double getMotorRevsPerWheelRev() {
        return this.motorRevsPerWheelRev;
    }

    private double wheelDiameterInches = 4.0;

    public void setWheelDiameterInches(double wheelDiameterInches) {
        this.wheelDiameterInches = wheelDiameterInches;
    }

    @Override
    public double getWheelDiameterInches() {
        return this.wheelDiameterInches;
    }

    private double maxSpeedRpm = 5000.0;

    public void setMaxSpeedRpm(double maxSpeedRpm) {
        this.maxSpeedRpm = maxSpeedRpm;
    }

    @Override
    public double getMaxSpeedRpm() {
        return this.maxSpeedRpm;
    }

}
