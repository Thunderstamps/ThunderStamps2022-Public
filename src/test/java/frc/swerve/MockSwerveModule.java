package frc.swerve;

import frc.robot.subsystems.swerve.ISwerveModule;
import frc.robot.subsystems.swerve.IVector2D;
import frc.robot.subsystems.swerve.Vector2D;

public class MockSwerveModule implements ISwerveModule {
    private boolean homed = true;
    public void setHomed(boolean homed){
        this.homed = homed;
    }

    @Override
    public boolean isHomed(){
        return this.homed;
    }

    private IVector2D velocityCommand_in_s_rad = null;

    @Override 
    public void executeVelocityMode(IVector2D velocityCommand_in_s_rad)
    {
        this.velocityCommand_in_s_rad = velocityCommand_in_s_rad;
    }

    public IVector2D getVelocityCommand_in_s_rad(){
        return this.velocityCommand_in_s_rad;
    }
    private IVector2D currentVelocity_in_s_rad = Vector2D.FromPolar(0.0, 0.0);

    public void setCurrentVelocity_in_s_rad(
        IVector2D currentVelocity_in_s_rad){
        this.currentVelocity_in_s_rad = currentVelocity_in_s_rad;
    }

    @Override
    public IVector2D getCurrentVelocity_in_s_rad(){
        return this.currentVelocity_in_s_rad;}

    private IVector2D modulePos_in = Vector2D.FromPolar(1.0, 0.0);

    public void setModulePos_in(
        IVector2D modulePos_in){
        this.modulePos_in = modulePos_in;
    }

    @Override
    public IVector2D getModulePos_in(){
        return modulePos_in;
    }

    private double moduleOrientation_rad;

    public void setModuleOrientation_rad(
        double moduleOrientation_rad){
        this.moduleOrientation_rad = moduleOrientation_rad;
    }

    @Override
    public double getModuleOrientation_rad(){
        return this.moduleOrientation_rad;
    }

    private double maxSpeed_in_s = 40000.0;

    public void setMaxSpeed_in_s(double maxSpeed_in_s){
        this.maxSpeed_in_s = maxSpeed_in_s;
    }

    @Override
    public double getMaxSpeed_in_s(){
        return this.maxSpeed_in_s;
    }

}