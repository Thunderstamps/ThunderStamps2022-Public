package frc.swerve;

import frc.robot.subsystems.swerve.SwerveUtil;
import org.junit.*;
import static org.junit.Assert.*;

public class TestSwerveUtil {

    @Test
    public void testSwerveUtil() {
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(5 * (Math.PI) / 2), ((Math.PI) / 2), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(0), 0, 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad((Math.PI) / 2), ((Math.PI) / 2), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad((Math.PI)), (-Math.PI), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad((3 * (Math.PI)) / 2), ((-1) * (Math.PI) / 2), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad((-1 * (Math.PI) / 2)), (-1 * (Math.PI) / 2), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad((-1 * (Math.PI))), ((-1) * Math.PI), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(-(3 * (Math.PI) / 2)), ((Math.PI) / 2), 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(-(2 * (Math.PI))), 0, 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(2 * (Math.PI)), 0, 0.1);
        assertEquals(SwerveUtil.limitAngleFromNegativePItoPI_rad(-5 * (Math.PI) / 2), (-(Math.PI) / 2), 0.1);
    }

    @Test
    public void Test_shortestDiffToTargetAngle_rad(){
        test_shortestDiffToTargetAngle_rad(0.0, 0.0, 0.0);
        test_shortestDiffToTargetAngle_rad(Math.PI/2, Math.PI/2, 0.0);
        test_shortestDiffToTargetAngle_rad(-Math.PI/2, -Math.PI/2, 0.0);
        test_shortestDiffToTargetAngle_rad(Math.PI, Math.PI, 0.0);
        test_shortestDiffToTargetAngle_rad(-Math.PI, -Math.PI, 0.0);
        test_shortestDiffToTargetAngle_rad(Math.PI, -Math.PI, 0.0);
        test_shortestDiffToTargetAngle_rad(-Math.PI, Math.PI, 0.0);

        test_shortestDiffToTargetAngle_deg(10.0, 20.0, 10.0);
        test_shortestDiffToTargetAngle_deg(-10.0, -20.0, -10.0);
        test_shortestDiffToTargetAngle_deg(-10.0, 10.0, 20.0);
        test_shortestDiffToTargetAngle_deg(170.0, -170.0, 20.0);
        test_shortestDiffToTargetAngle_deg(180.0, -170.0, 10.0);
        test_shortestDiffToTargetAngle_deg(170.0, -180.0, 10.0);
        test_shortestDiffToTargetAngle_deg(180.0, 170.0, -10.0);
        test_shortestDiffToTargetAngle_deg(-170.0, -180.0, -10.0);
    }
    private void test_shortestDiffToTargetAngle_rad(
            double currentAngle_rad,
            double targetAngle_rad,
            double expected_rad) {
        double result = SwerveUtil.shortestDiffToTargetAngle_rad(currentAngle_rad, targetAngle_rad);
        assertEquals(expected_rad, result, Math.toRadians(0.01));
    }

    private void test_shortestDiffToTargetAngle_deg(
            double currentAngle_deg,
            double targetAngle_deg,
            double expected_deg) {
        test_shortestDiffToTargetAngle_rad(Math.toRadians(currentAngle_deg), Math.toRadians(targetAngle_deg),
                Math.toRadians(expected_deg));
    }



}
