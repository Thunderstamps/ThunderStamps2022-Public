package frc.swerve;
import frc.robot.subsystems.swerve.*;


public class MockGyro implements IGyro {
    private double fieldOrientation_rad = 0.0;

    @Override
    public double getFieldOrientation_rad() {
        return this.fieldOrientation_rad;
    }
    public void setFieldOrientation_rad(
            double fieldOrientation_rad) {
        this.fieldOrientation_rad = fieldOrientation_rad;
    }

    private double rotationRate_rad_s = 0.0;

    @Override
    public double getRotationRate_rad_s() {
        return this.rotationRate_rad_s;
    }
    public void setRotationRate_rad_s(
            double rotationRate_rad_s) {
        this.rotationRate_rad_s = rotationRate_rad_s;
    }

    private boolean enabled = true;

    @Override
    public boolean getEnabled() { return this.enabled; }
    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void resetOffsetToZero_rad() { } // not needed

    @Override
    public void adjustOffset_rad(double adjustment_rad) { }
    
    @Override
    public double getZAccel_G() {
        return 0;
    }


}
