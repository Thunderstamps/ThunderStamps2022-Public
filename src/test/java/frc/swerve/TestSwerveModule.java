package frc.swerve;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.robot.subsystems.swerve.IDriveController;
import frc.robot.subsystems.swerve.ISteeringController;
import frc.robot.subsystems.swerve.IVector2D;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.Vector2D;

public class TestSwerveModule {

    @Test
    public void Test_getMaxSpeed_in_s() {
        test_getMaxSpeed_in_s(60.0, 1.0, 1.0 / Math.PI, 1.0);
        test_getMaxSpeed_in_s(5000.0, 8.0, 4.0, 130.9);
        // checks a zero gear ratio returns 0 rather than divide by 0
        test_getMaxSpeed_in_s(5000.0, 0.0, 4.0, 0.0);
    }

    private void test_getMaxSpeed_in_s(
            double maxMotorRpm,
            double motorRevsPerWheelRev,
            double wheelDiameterInches,
            double expected) {
        MockDriveController driveController = new MockDriveController();
        driveController.setMaxSpeedRpm(maxMotorRpm);
        driveController.setMotorRevsPerWheelRev(motorRevsPerWheelRev);
        driveController.setWheelDiameterInches(wheelDiameterInches);

        SwerveModule test = makeSwerveModule(driveController);
        double actual = test.getMaxSpeed_in_s();
        assertEquals(expected, actual, 0.01);
    }

    private SwerveModule makeSwerveModule(IDriveController driveController) {
        ISteeringController steeringController = new MockSteeringController();
        IVector2D modulePosition = Vector2D.FromPolar(1.0, 0);
        double moduleOrientation = 0.0;
        return new SwerveModule(steeringController, driveController, modulePosition, moduleOrientation);
    }

    @Test
    public void Test_getCurrentVelocity_in_s_rad() {
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, 1.0, 0.0);
        test_getCurrentVelocity_in_s_rad(120.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, 2.0, 0.0);
        test_getCurrentVelocity_in_s_rad(60.0, 2.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, 0.5, 0.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 2.0 / Math.PI, 0.0, 1.0, 0.0, 2.0, 0.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, 0.25, 1.0, 0.0, 1.0, Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, -0.25, 1.0, 0.0, 1.0, -Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(-60.0, 1.0, 1.0 / Math.PI, 0.25, 1.0, 0.0, 1.0, -Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(-60.0, 1.0, 1.0 / Math.PI, -0.25, 1.0, 0.0, 1.0, Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, 0.75, 1.0, 0.0, 1.0, -Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, -0.75, 1.0, 0.0, 1.0, Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, 1.75, 1.0, 0.0, 1.0, -Math.PI / 2.);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, -1.75, 1.0, 0.0, 1.0, Math.PI / 2.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, 0.5, 3.0, 0.0, 1.0, Math.PI / 3.0);
        test_getCurrentVelocity_in_s_rad(60.0, 1.0, 1.0 / Math.PI, -0.5, 3.0, 0.0, 1.0, -Math.PI / 3.0);
    }

    private void test_getCurrentVelocity_in_s_rad(
            double actualDriveMotorSpeedRpm,
            double driveControllerMotorRevsPerWheelRev,
            double driveControllerWheelDiameterInches,
            double actualSteeringMotorPositionRevs,
            double steeringControllerMotorRevsPerSteeringRev,
            double swerveModuleOrientation_rad,
            double expectedMagnitude_in_s,
            double expectedDirection_rad) {

        MockDriveController driveController = new MockDriveController();
        driveController.setMotorRpm(actualDriveMotorSpeedRpm);
        ;
        driveController.setMotorRevsPerWheelRev(driveControllerMotorRevsPerWheelRev);
        driveController.setWheelDiameterInches(driveControllerWheelDiameterInches);

        MockSteeringController steeringController = new MockSteeringController();
        steeringController.setMotorRevCount(actualSteeringMotorPositionRevs);
        steeringController.setMotorRevsPerSteeringRev(
                steeringControllerMotorRevsPerSteeringRev);

        SwerveModule test = new SwerveModule(
                steeringController,
                driveController,
                Vector2D.FromXY(1.0, 1.0),
                swerveModuleOrientation_rad);

        IVector2D actual = test.getCurrentVelocity_in_s_rad();
        assertEquals(expectedMagnitude_in_s, actual.getMagnitude(), 0.01);
        assertEquals(expectedDirection_rad, actual.getAngleRadians(), 0.01);
    }

    @Test
    public void Test_execute() {
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, 0.0, 60.0);
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, 0.0, 2.0, 0.0, 0.0, 120.0);
        test_execute(1.0, 0.0, 2.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, 0.0, 120.0);
        test_execute(1.0, 0.0, 1.0, 2.0 / Math.PI, 0.0, 1.0, 0.0, 0.0, 30.0);
        for (double r = -2.0; r <= 2.0; r += 1.0) {
            test_execute(1.0, r + 0.5, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.5, -60.0);
            test_execute(1.0, r + 0.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(1.0, r - 0.5, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.5, -60.0);
        }
        for (double r = -20.0; r <= 20.0; r += 10.0) {
            test_execute(10.0, r + 5.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 5.0, -60.0);
            test_execute(10.0, r + 0.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(10.0, r - 5.0, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 5.0, -60.0);
        }
        for (double r = -2.0; r <= 2.0; r += 1.0) {
            test_execute(1.0, r + 0.7, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.5, -60.0);
            test_execute(1.0, r + 0.6, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.5, -60.0);
            test_execute(1.0, r + 0.4, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.5, -60.0);
            test_execute(1.0, r + 0.3, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.5, -60.0);

            test_execute(1.0, r + 0.2, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(1.0, r + 0.1, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(1.0, r - 0.1, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(1.0, r - 0.2, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);

            test_execute(1.0, r - 0.3, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.5, -60.0);
            test_execute(1.0, r - 0.4, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.5, -60.0);
            test_execute(1.0, r - 0.6, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.5, -60.0);
            test_execute(1.0, r - 0.7, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.5, -60.0);
        }
        for (double r = -2.0; r <= 2.0; r += 1.0) {
            test_execute(1.0, r + 0.25, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r + 0.0, 60.0);
            test_execute(1.0, r - 0.25, 1.0, 1.0 / Math.PI, 0.0, 1.0, 0.0, r - 0.0, 60.0);
        }
        double delta_rad = 0.1;
        double delta_rev = delta_rad / (2.0 * Math.PI);
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, delta_rad, 1.0, 0.0, -delta_rev, 60.0);
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, -delta_rad, 1.0, 0.0, delta_rev, 60.0);
        double d2_rad = Math.toRadians(95.0);
        double d2_rev = Math.toRadians(85.0) / (2.0 * Math.PI);
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, d2_rad, 1.0, 0.0, d2_rev, -60.0);
        test_execute(1.0, 0.0, 1.0, 1.0 / Math.PI, -d2_rad, 1.0, 0.0, -d2_rev, -60.0);
        test_execute(9.0, 9.0 / 8.0, 8.0, 4.0, 0.0, 5.3, -Math.PI / 2.0, 9.0 / 4.0, -202.445);
    }

    private void test_execute(
            double steeringControllerMotorRevsPerSteeringRev,
            double steeringControllerMotorRevCount,
            double driveControllerMotorRevsPerWheelRev,
            double driveControllerWheelDiameterInches,
            double swerveModuleOrientationradians,
            double commandVelocityMagnitudeInchesPerSecond,
            double commandDirectionRadians,
            double expectedSteerinfTargetMotorRevs,
            double expectedDriveTargetMotorRpm) {

        MockSteeringController steeringController = new MockSteeringController();
        steeringController.setMotorRevCount(steeringControllerMotorRevCount);
        steeringController.setMotorRevsPerSteeringRev(steeringControllerMotorRevsPerSteeringRev);

        MockDriveController driveController = new MockDriveController();
        driveController.setMotorRevsPerWheelRev(driveControllerMotorRevsPerWheelRev);
        driveController.setWheelDiameterInches(driveControllerWheelDiameterInches);

        SwerveModule test = new SwerveModule(steeringController, driveController, Vector2D.FromXY(1.0, 1.0),
                swerveModuleOrientationradians);

        IVector2D velocityCommand = Vector2D.FromPolar(commandVelocityMagnitudeInchesPerSecond,
                commandDirectionRadians);

        test.executeVelocityMode(velocityCommand);

        assertEquals(expectedSteerinfTargetMotorRevs, steeringController.getTargetMotorRev(), 0.001);
        assertEquals(expectedDriveTargetMotorRpm, driveController.getTargetMotorRpm(), 0.001);
    }

}
