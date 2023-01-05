package frc.swerve;

import frc.robot.subsystems.swerve.*;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

public class TestFieldOrientedSwerve {

    @Test
    public void Test_execute_robotOriented_passThrough() {
        test_execute_robotOriented_passThrough(0.0, 0.0, 0.0);
        test_execute_robotOriented_passThrough(10.0, 0.0, 0.0);
        test_execute_robotOriented_passThrough(0.0, -5.0, 0.0);
        test_execute_robotOriented_passThrough(0.0, 0.0, 1.0);
        test_execute_robotOriented_passThrough(-8.2, 1.3, -0.86);
    }

    private void test_execute_robotOriented_passThrough(
            double translationCommandX_in_s,
            double translationCommandY_in_s,
            double rotationRate_rad_s) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double MAX_ROTATION_RATE_RAD_S = 100.0; // very high
        final double SCAN_TIME_S = 0.02; // typical
        final double ROTATION_P = 0.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                MAX_ACCELERATION_IN_S2,
                MAX_ROTATION_RATE_RAD_S,
                SCAN_TIME_S,
                ROTATION_P);

        Vector2D translationCommand_in_s_rad = Vector2D.FromXY(
                translationCommandX_in_s, translationCommandY_in_s);

        test.execute(
                Vector2D.FromXY(0.0, 0.0),
                0.0,
                translationCommand_in_s_rad,
                rotationRate_rad_s);

        IVector2D translationResult = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        double resultRotationRate = robotOrientedSwerve.getTargetRotationRate_rad_s();

        assertNotNull(translationResult);
        assertEquals(translationCommandX_in_s, translationResult.getX(), 0.001);
        assertEquals(translationCommandY_in_s, translationResult.getY(), 0.001);
        assertEquals(rotationRate_rad_s, resultRotationRate, 0.001);
    }

    @Test
    public void Test_execute_fieldOrientedConversion() {
        test_execute_fieldOrientedConversion(0.0, 0.0, 0.0, 0.0, 0.0);
        test_execute_fieldOrientedConversion(10.0, 0.0, 0.0, 10.0, 0.0);
        test_execute_fieldOrientedConversion(0.0, -5.0, 0.0, 0.0, -5.0);
        test_execute_fieldOrientedConversion(-2.3, 9.81, 0.0, -2.3, 9.81);
        test_execute_fieldOrientedConversion(5.0, 0.0, Math.PI / 2.0, 0.0, -5.0);
        test_execute_fieldOrientedConversion(5.0, 0.0, -Math.PI / 2.0, 0.0, 5.0);
        test_execute_fieldOrientedConversion(5.0, 0.0, Math.PI, -5.0, 0.0);
        test_execute_fieldOrientedConversion(5.0, 0.0, -Math.PI, -5.0, 0.0);
        test_execute_fieldOrientedConversion(3.0, 4.0, -Math.PI / 2.0, -4.0, 3.0);
        test_execute_fieldOrientedConversion(10.0, 0.0, Math.PI / 4.0, 7.071, -7.071);
        test_execute_fieldOrientedConversion(10.0, 0.0, -Math.PI / 4.0, 7.071, 7.071);
    }

    private void test_execute_fieldOrientedConversion(
            double fieldOrientedTranslationCommandX_in_s,
            double fieldOrientedTranslationCommandY_in_s,
            double gyroFieldOrientation_rad,
            double expectedTranslationX_rad_s,
            double expectedTranslationY_rad_s) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();
        gyro.setFieldOrientation_rad(gyroFieldOrientation_rad);

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double MAX_ROTATION_RATE_RAD_S = 100.0; // very high
        final double SCAN_TIME_S = 0.02; // typical
        final double ROTATION_P = 0.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                MAX_ACCELERATION_IN_S2,
                MAX_ROTATION_RATE_RAD_S,
                SCAN_TIME_S,
                ROTATION_P);

        Vector2D translationCommand_in_s_rad = Vector2D.FromXY(
                fieldOrientedTranslationCommandX_in_s,
                fieldOrientedTranslationCommandY_in_s);
        test.execute(
                translationCommand_in_s_rad,
                gyroFieldOrientation_rad, // tell it to face the direction it's facing
                Vector2D.FromXY(0.0, 0.0),
                0.0);

        IVector2D translationResult = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        double resultRotationRate = robotOrientedSwerve.getTargetRotationRate_rad_s();

        assertNotNull(translationResult);
        assertEquals(expectedTranslationX_rad_s, translationResult.getX(), 0.001);
        assertEquals(expectedTranslationY_rad_s, translationResult.getY(), 0.001);
        assertEquals(0.0, resultRotationRate, 0.001);
    }

    @Test
    public void Test_execute_rotationPID() {
        test_execute_rotationPID(0.0, 0.0, 1.0, Math.PI, 0.0);
        test_execute_rotationPID(Math.PI / 4.0, 0.0, 1.0, Math.PI, Math.PI / 4.0);
        test_execute_rotationPID(-Math.PI / 4.0, 0.0, 1.0, Math.PI, -Math.PI / 4.0);
        test_execute_rotationPID(Math.PI / 4.0, 0.0, 0.5, Math.PI, Math.PI / 8.0);
        test_execute_rotationPID(-Math.PI / 4.0, 0.0, 0.5, Math.PI, -Math.PI / 8.0);
        test_execute_rotationPID(Math.PI / 2.0, 0.0, 1.0, Math.PI / 4.0, Math.PI / 4.0);
        test_execute_rotationPID(-Math.PI / 2.0, 0.0, 1.0, Math.PI / 4.0, -Math.PI / 4.0);
        test_execute_rotationPID(-0.75 * Math.PI, 0.75 * Math.PI, 2.0, Math.PI, Math.PI);
        test_execute_rotationPID(0.75 * Math.PI, -0.75 * Math.PI, 2.0, Math.PI, -Math.PI);

    }

    @Test
    public void Test_execute_combined() {
        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();
        gyro.setFieldOrientation_rad(0.0);

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double MAX_ROTATION_RATE_RAD_S = 100.0; // very high
        final double SCAN_TIME_S = 0.02; // typical
        final double ROTATION_P = 1.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                MAX_ACCELERATION_IN_S2,
                MAX_ROTATION_RATE_RAD_S,
                SCAN_TIME_S,
                ROTATION_P);

        test.execute(
                Vector2D.FromXY(-1.3, 2.8),
                Math.PI / 2.0,
                Vector2D.FromXY(6.55, -7.38),
                -Math.PI / 4.0);

        IVector2D translationResult = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        double resultRotationRate = robotOrientedSwerve.getTargetRotationRate_rad_s();

        double expectedRotationRate = Math.PI / 2.0 - Math.PI / 4.0;
        assertNotNull(translationResult);
        assertEquals(6.55 - 1.3, translationResult.getX(), 0.001);
        assertEquals(2.8 - 7.38, translationResult.getY(), 0.001);
        assertEquals(expectedRotationRate, resultRotationRate, 0.001);
    }

    @Test
    public void Test_execute_maximumRotationRate() {
        test_execute_maximumRotationRate(0.0, 0.0, 1.0, 0.0);
        test_execute_maximumRotationRate(1.0, -1.0, 1.0, 0.0);
        test_execute_maximumRotationRate(1.0, 0.0, 1.0, 1.0);
        test_execute_maximumRotationRate(-1.0, 0.0, 1.0, -1.0);
        test_execute_maximumRotationRate(0.0, 1.0, 1.0, 1.0);
        test_execute_maximumRotationRate(0.0, -1.0, 1.0, -1.0);
        test_execute_maximumRotationRate(1.0, 0.0, 0.5, 0.5);
        test_execute_maximumRotationRate(-1.0, 0.0, 0.5, -0.5);
        test_execute_maximumRotationRate(0.0, 1.0, 0.5, 0.5);
        test_execute_maximumRotationRate(0.0, -1.0, 0.5, -0.5);
        test_execute_maximumRotationRate(1.0, 1.0, 3.0, 2.0);
        test_execute_maximumRotationRate(-1.0, -1.0, 3.0, -2.0);
        test_execute_maximumRotationRate(1.0, 1.0, 3.0, 2.0);
        test_execute_maximumRotationRate(-1.0, -1.0, 3.0, -2.0);
        test_execute_maximumRotationRate(1.0, 1.0, 1.0, 1.0);
        test_execute_maximumRotationRate(-1.0, -1.0, 1.0, -1.0);
        test_execute_maximumRotationRate(1.0, 1.0, 1.0, 1.0);
        test_execute_maximumRotationRate(-1.0, -1.0, 1.0, -1.0);
    }

    private void test_execute_maximumRotationRate(
            double fieldRotation_rad_s,
            double robotRotation_rad_s,
            double maxRotationRate_rad_s,
            double expectedRotationRate_rad_s) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double SCAN_TIME_S = 0.02; // typical
        final double ROTATION_P = 1.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                MAX_ACCELERATION_IN_S2,
                maxRotationRate_rad_s,
                SCAN_TIME_S,
                ROTATION_P);

        test.execute(
                Vector2D.FromXY(0.0, 0.0),
                fieldRotation_rad_s,
                Vector2D.FromXY(0.0, 0.0),
                robotRotation_rad_s);

        double resultRotationRate = robotOrientedSwerve.getTargetRotationRate_rad_s();
        assertEquals(expectedRotationRate_rad_s, resultRotationRate, 0.001);
    }

    private void test_execute_rotationPID(
            double fieldOrientedHeadingCommand_rad,
            double gyroFieldOrientation_rad,
            double rotationP,
            double maxRotationRate_rad_s,
            double expectedRotationRate_rad_s) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();
        gyro.setFieldOrientation_rad(gyroFieldOrientation_rad);

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double SCAN_TIME_S = 0.02; // typical

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                MAX_ACCELERATION_IN_S2,
                maxRotationRate_rad_s,
                SCAN_TIME_S,
                rotationP);

        test.execute(
                Vector2D.FromXY(0.0, 0.0),
                fieldOrientedHeadingCommand_rad,
                Vector2D.FromXY(0.0, 0.0),
                0.0);

        IVector2D translationResult = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        double resultRotationRate = robotOrientedSwerve.getTargetRotationRate_rad_s();

        assertNotNull(translationResult);
        assertEquals(0.0, translationResult.getX(), 0.001);
        assertEquals(0.0, translationResult.getY(), 0.001);
        assertEquals(expectedRotationRate_rad_s, resultRotationRate, 0.001);
    }

    @Test
    public void Test_execute_maximumSpeed() {
        test_execute_maximumSpeed(0.0, 0.0, 10.0, 0.0);
        test_execute_maximumSpeed(1.0, -1.0, 10.0, 0.0);
        test_execute_maximumSpeed(1.0, 1.0, 10.0, 2.0);
        test_execute_maximumSpeed(-1.0, -1.0, 10.0, -2.0);
        test_execute_maximumSpeed(11.0, 0.0, 10.0, 10.0);
        test_execute_maximumSpeed(0.0, 11.0, 10.0, 10.0);
        test_execute_maximumSpeed(-11.0, 0.0, 10.0, -10.0);
        test_execute_maximumSpeed(0.0, -11.0, 10.0, -10.0);
        test_execute_maximumSpeed(5.0, 6.0, 10.0, 10.0);
        test_execute_maximumSpeed(-5.0, -6.0, 10.0, -10.0);
        test_execute_maximumSpeed(5.0, -6.0, 10.0, -1.0);
        test_execute_maximumSpeed(-5.0, 6.0, 10.0, 1.0);
    }

    private void test_execute_maximumSpeed(
            double fieldSpeed_in_s,
            double robotSpeed_in_s,
            double maxSpeed_in_s,
            double expectedSpeed_in_s) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();

        final double MAX_ACCELERATION_IN_S2 = 1000000.0; // very high
        final double MAX_ROTATION_RATE_RAD_S = 100.0; // very high
        final double SCAN_TIME_S = 0.02; // typical
        final double ROTATION_P = 1.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                maxSpeed_in_s,
                MAX_ACCELERATION_IN_S2,
                MAX_ROTATION_RATE_RAD_S,
                SCAN_TIME_S,
                ROTATION_P);

        test.execute(
                Vector2D.FromXY(fieldSpeed_in_s, 0.0),
                0.0,
                Vector2D.FromXY(robotSpeed_in_s, 0.0),
                0.0);

        IVector2D translationResult = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        assertNotNull(translationResult);
        assertEquals(expectedSpeed_in_s, translationResult.getX(), 0.001);
        assertEquals(0.0, translationResult.getY(), 0.001);
    }

    @Test
    public void Test_execute_maxAcceleration() {
        test_execute_maxAcceleration(100.0, 0.02, 0.0, 0.0, 3.0, 0.0, 0.0, 2.0, 3.0, 0.0);
        test_execute_maxAcceleration(50.0, 0.02, 0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 2.0, 0.0);
        test_execute_maxAcceleration(100.0, 0.01, 0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 2.0, 0.0);
        test_execute_maxAcceleration(100.0, 0.01, 1.0, 0.0, 1.0, 0.0, Math.PI / 2.0, 1.0, 1.414, -Math.PI / 4.0);
    }

    private void test_execute_maxAcceleration(
            double maxAcceleration_in_s2,
            double scanTime_s,
            double fieldOrientedCommand_in_s,
            double fieldOrientedCommand_rad,
            double robotOrientedCommand_in_s,
            double robotOrientedCommand_rad,
            double gyroFieldOrientation_rad,
            double expectedMagnitude1_in_s,
            double expectedMagnitude2_in_s,
            double expectedAngle_rad) {

        MockRobotOrientedSwerve robotOrientedSwerve = new MockRobotOrientedSwerve();
        MockGyro gyro = new MockGyro();
        gyro.setFieldOrientation_rad(gyroFieldOrientation_rad);

        final double MAX_SPEED_IN_S = 40000.0; // about 1 km/s
        final double MAX_ROTATION_RATE_RAD_S = 100.0; // very high
        final double ROTATION_P = 1.0;

        FieldOrientedSwerve test = new FieldOrientedSwerve(
                robotOrientedSwerve,
                gyro,
                MAX_SPEED_IN_S,
                maxAcceleration_in_s2,
                MAX_ROTATION_RATE_RAD_S,
                scanTime_s,
                ROTATION_P);

        Vector2D fieldOrientedCommand = Vector2D.FromPolar(
                fieldOrientedCommand_in_s, fieldOrientedCommand_rad);
        Vector2D robotOrientedCommand = Vector2D.FromPolar(
                robotOrientedCommand_in_s, robotOrientedCommand_rad);

        // First call to execute()
        test.execute(
                fieldOrientedCommand,
                gyroFieldOrientation_rad,
                robotOrientedCommand,
                0.0);
        IVector2D translationResult1 = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        assertNotNull(translationResult1);
        assertEquals(expectedMagnitude1_in_s,
                translationResult1.getMagnitude(), 0.001);
        assertEquals(expectedAngle_rad,
                translationResult1.getAngleRadians(), 0.001);

        // Second call to execute()
        test.execute(
                fieldOrientedCommand,
                gyroFieldOrientation_rad,
                robotOrientedCommand,
                0.0);

        IVector2D translationResult2 = robotOrientedSwerve.getTranslationCommand_in_s_rad();
        assertNotNull(translationResult2);
        assertEquals(expectedMagnitude2_in_s,
                translationResult2.getMagnitude(), 0.001);
        assertEquals(expectedAngle_rad,
                translationResult2.getAngleRadians(), 0.001);
    }

}
