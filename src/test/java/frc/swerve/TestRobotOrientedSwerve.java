package frc.swerve;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.ArrayList;

import org.junit.Test;

import frc.robot.subsystems.swerve.ISwerveModule;
import frc.robot.subsystems.swerve.IVector2D;
import frc.robot.subsystems.swerve.RobotOrientedSwerve;
import frc.robot.subsystems.swerve.Vector2D;

public class TestRobotOrientedSwerve {

    @Test
    public void Test_execute_single_swerve_module() {
        // Just test translation by itself
        test_execute_single_swerve_module(10.0, 10.0, 5.0, 0.0, 0.0, 5.0, 0.0);
        test_execute_single_swerve_module(-5.0, 20.0, 8.0, -3.5, 0.0, 8.0, -3.5);
        test_execute_single_swerve_module(0.0, -3.33, -2.0, 1.5, 0.0, -2.0, 1.5);
        test_execute_single_swerve_module(-1.0, -1.0, 0.0, 11.7, 0.0, 0.0, 11.7);

        // Now test rotation by itself
        test_execute_single_swerve_module(0.0, 10.0, 0.0, 0.0, 1.0, -10.0, 0.0);
        test_execute_single_swerve_module(0.0, -10.0, 0.0, 0.0, 1.0, 10.0, 0.0);
        test_execute_single_swerve_module(0.0, 5.0, 0.0, 0.0, 1.0, -5.0, 0.0);
        test_execute_single_swerve_module(0.0, -5.0, 0.0, 0.0, 1.0, 5.0, 0.0);
        test_execute_single_swerve_module(0.0, 5.0, 0.0, 0.0, 2.0, -10.0, 0.0);
        test_execute_single_swerve_module(0.0, -5.0, 0.0, 0.0, 2.0, 10.0, 0.0);
        test_execute_single_swerve_module(0.0, 5.0, 0.0, 0.0, -2.0, 10.0, 0.0);
        test_execute_single_swerve_module(0.0, -5.0, 0.0, 0.0, -2.0, -10.0, 0.0);
        test_execute_single_swerve_module(10.0, 0.0, 0.0, 0.0, 1.0, 0.0, 10.0);
        test_execute_single_swerve_module(-10.0, 0.0, 0.0, 0.0, 1.0, 0.0, -10.0);

        test_execute_single_swerve_module(7.071, 7.071, 0.0, 0.0, 1.0, -7.071, 7.071);
        test_execute_single_swerve_module(-7.071, 7.071, 0.0, 0.0, 1.0, -7.071, -7.071);
        test_execute_single_swerve_module(-7.071, -7.071, 0.0, 0.0, 1.0, 7.071, -7.071);
        test_execute_single_swerve_module(7.071, -7.071, 0.0, 0.0, 1.0, 7.071, 7.071);
        test_execute_single_swerve_module(7.071, 7.071, 0.0, 0.0, -1.0, 7.071, -7.071);
        test_execute_single_swerve_module(-7.071, 7.071, 0.0, 0.0, -1.0, 7.071, 7.071);
        test_execute_single_swerve_module(-7.071, -7.071, 0.0, 0.0, -1.0, -7.071, 7.071);
        test_execute_single_swerve_module(7.071, -7.071, 0.0, 0.0, -1.0, -7.071, -7.071);

        test_execute_single_swerve_module(3.0, 4.0, 0.0, 0.0, 2.0, -8.0, 6.0);
        test_execute_single_swerve_module(3.0, 4.0, 12.5, -3.2, 2.0, 4.5, 2.8);
    }

    private void test_execute_single_swerve_module(
            double modulePosX_in,
            double modulePosY_in,
            double translationCommandX_in_s,
            double translationCommandY_in_s,
            double rotationCommand_rad_s,
            double expectedVelocityCommandX_in_s,
            double expectedVelocityCommandY_in_s) {
        MockSwerveModule swerveModule = new MockSwerveModule();
        swerveModule.setMaxSpeed_in_s(40000.0);
        assertTrue(swerveModule.isHomed());
        swerveModule.setModulePos_in(Vector2D.FromXY(modulePosX_in, modulePosY_in));

        ArrayList<ISwerveModule> swerveModules = new ArrayList<>(Arrays.asList(swerveModule));

        RobotOrientedSwerve test = new RobotOrientedSwerve(swerveModules);

        IVector2D translationCommand_in_s_rad = Vector2D.FromXY(translationCommandX_in_s, translationCommandY_in_s);
        test.execute(translationCommand_in_s_rad, rotationCommand_rad_s);

        IVector2D result = swerveModule.getVelocityCommand_in_s_rad();
        assertNotNull(result);
        assertEquals(expectedVelocityCommandX_in_s, result.getX(), 0.01);
        assertEquals(expectedVelocityCommandY_in_s, result.getY(), 0.01);

    }

    @Test
    public void Test_Execute_isHomed(){
        test_execute_isHomed(false, false, false);
        test_execute_isHomed(true, false, false);
        test_execute_isHomed(false, true, false);
        test_execute_isHomed(true, true, true);
    }

    private void test_execute_isHomed(boolean swerveModule1isHomed,
    boolean swerveModule2isHomed,
    boolean setsVelocityCommand){
        MockSwerveModule swerveModule1 = new MockSwerveModule();
        swerveModule1.setHomed(swerveModule1isHomed);
        MockSwerveModule swerveModule2 = new MockSwerveModule();
        swerveModule2.setHomed(swerveModule2isHomed);

        ArrayList<ISwerveModule> swerveModules = new ArrayList<>(Arrays.asList(swerveModule1, swerveModule2));
        RobotOrientedSwerve test = new RobotOrientedSwerve(swerveModules);

        test.execute(Vector2D.FromXY(0.0, 0.0), 0.0);

        IVector2D result1 = swerveModule1.getVelocityCommand_in_s_rad();
        IVector2D result2 = swerveModule2.getVelocityCommand_in_s_rad();
        if(setsVelocityCommand){
            assertNotNull(result1);
            assertNotNull(result2);
        }
        else{
            assertNull(result1);
            assertNull(result2);
        }
    }

    @Test
    public void Test_execut_MaxSpeed(){
    MockSwerveModule swerveModuleLeft = new MockSwerveModule();
    swerveModuleLeft.setMaxSpeed_in_s(10.0);
    swerveModuleLeft.setModulePos_in(
        Vector2D.FromXY(0.0, 10.0)
    );
    MockSwerveModule swerveModuleFront = new MockSwerveModule();
    swerveModuleFront.setMaxSpeed_in_s(10.0);
    swerveModuleFront.setModulePos_in(
        Vector2D.FromXY(10.0, 0.0)
    );
    MockSwerveModule swerveModuleRight = new MockSwerveModule();
    swerveModuleRight.setMaxSpeed_in_s(10.0);
    swerveModuleRight.setModulePos_in(
        Vector2D.FromXY(0.0, -10.0)
    );
    
    ArrayList<ISwerveModule> swerveModules = new ArrayList<>(
        Arrays.asList(swerveModuleLeft, swerveModuleFront, swerveModuleRight));
    RobotOrientedSwerve test = new RobotOrientedSwerve(swerveModules);

    test.execute(Vector2D.FromXY(10.0, 0.0), 1.0);

    IVector2D resultLeft = swerveModuleLeft.getVelocityCommand_in_s_rad();
    assertEquals(resultLeft.getMagnitude(), 0.0, 0.001);

    IVector2D resultFront = swerveModuleFront.getVelocityCommand_in_s_rad();
    assertEquals(resultFront.getMagnitude(), 7.071, 0.001);
    assertEquals(resultFront.getAngleRadians(), Math.toRadians(45.0), 0.001);
    
    IVector2D resultRight = swerveModuleRight.getVelocityCommand_in_s_rad();
    assertEquals(resultRight.getMagnitude(), 10.0, 0.001);
    assertEquals(resultRight.getAngleRadians(), 0.0, 0.001);

    }

}
