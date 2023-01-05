package frc.swerve;

import org.junit.Test;

import frc.robot.subsystems.swerve.Vector2D;

import static org.junit.Assert.*;

public class TestVector2D {
    @Test
    public void Can_construct_FromXY() {
        Can_construct_fromXY(3, 4, 5, 53.13);
        Can_construct_fromXY(4, 3, 5, 36.87);
        Can_construct_fromXY(1, 1, Math.sqrt(2.0), 45);



    }
    private void Can_construct_fromXY(double x, double y, double magnitude, double angleDegrees) {
        Vector2D test = Vector2D.FromXY(x, y);


        assertEquals(x, test.getX(), 0.0);
        assertEquals(y, test.getY(), 0.0);
        assertEquals(magnitude, test.getMagnitude(), 0.0);
        assertEquals(angleDegrees, Math.toDegrees(test.getAngleRadians()), 0.02);


    }
    @Test
    public void can_construct_FromPolar() {
        can_construct_fromPolar(1, 1, Math.sqrt(2.0), Math.toRadians(45));

    }

    private void can_construct_fromPolar(double x, double y, double magnitude, double angleRadians) {
        Vector2D test = Vector2D.FromPolar(magnitude, angleRadians);


        assertEquals(x, test.getX(), 0.01);
        assertEquals(y, test.getY(), 0.01);
        assertEquals(magnitude, test.getMagnitude(), 0.0);
        assertEquals(angleRadians, test.getAngleRadians(), 0.0);


    }
}
