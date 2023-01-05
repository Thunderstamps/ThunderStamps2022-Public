package frc.swerve;

import frc.robot.subsystems.swerve.IRobotOrientedSwerve;
import frc.robot.subsystems.swerve.IVector2D;


public class MockRobotOrientedSwerve implements IRobotOrientedSwerve {
    private IVector2D translationCommand_in_s_rad;
    private double targetRotationRate_rad_s = 0.0;

    @Override
    public void execute(
            IVector2D translationCommand_in_s_rad, 
            double targetRotationRate_rad_s) {

        this.translationCommand_in_s_rad = translationCommand_in_s_rad;
        this.targetRotationRate_rad_s = targetRotationRate_rad_s;
    }

    public IVector2D getTranslationCommand_in_s_rad() {
        return this.translationCommand_in_s_rad;
    }

    public double getTargetRotationRate_rad_s() {
        return this.targetRotationRate_rad_s;
    }
}
