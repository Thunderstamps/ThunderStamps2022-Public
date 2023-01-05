package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavXMxpGyro implements IGyro {

    private AHRS gyro;
    private boolean enabled = true;
    private double offsetToZero_rad;

    public NavXMxpGyro() {
        this.gyro = new AHRS(SPI.Port.kMXP);
    }

    public double getFieldOrientation_rad() {
        double rawFieldOrientation_rad = Math.toRadians(-gyro.getYaw()) - offsetToZero_rad;
        return SwerveUtil.limitAngleFromNegativePItoPI_rad(rawFieldOrientation_rad);
    }

    public double getRotationRate_rad_s() {
        double rate = Math.toRadians(-gyro.getRate());
        return rate;
    }

    public void resetOffsetToZero_rad() {
        gyro.reset();
        gyro.zeroYaw();
        this.offsetToZero_rad = 0;
    }

    public void adjustOffset_rad(double adjustment_rad) {
        this.offsetToZero_rad += adjustment_rad;
    }

    public boolean getEnabled() {
        return this.enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public double getZAccel_G() {
        return this.gyro.getWorldLinearAccelZ();
    }

}