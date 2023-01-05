package frc.robot;

import java.sql.Time;
import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.grappler.GrapplerLockCommand;
import frc.robot.subsystems.grappler.GrapplerUnlockCommand;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.subsystems.vacuum.*;
import frc.robot.teleop.*;

public class NetworkTableComms {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable smartDashboard;

    private final NetworkTableEntry winch32Position_in;
    private final NetworkTableEntry winch33Position_in;
    private final NetworkTableEntry winch32AtTop;
    private final NetworkTableEntry winch33AtTop;
    private final NetworkTableEntry winch32PositionError;
    private final NetworkTableEntry winch33PositionError;
    private final NetworkTableEntry x;
    private final NetworkTableEntry y;
    private final NetworkTableEntry gyroEnabled;
    private final NetworkTableEntry vacSensor;
    private final NetworkTableEntry feederSpeed;
    private final NetworkTableEntry flywheelSpeed;
    private final NetworkTableEntry cameraSelection;
    private final NetworkTable fromPython;
    private final NetworkTableEntry aprilTagPresent;
    private final NetworkTableEntry aprilTagX;
    private final NetworkTableEntry aprilTagY;
    private final NetworkTableEntry delay;
    private final NetworkTableEntry systemTime;
    

    public NetworkTableComms() {

        this.smartDashboard = inst.getTable("SmartDashboard");
        this.fromPython = inst.getTable("fromPython");
        
        this.aprilTagPresent = fromPython.getEntry("AprilTagPresent");
        this.aprilTagX = fromPython.getEntry("AprilTagX");
        this.aprilTagY = fromPython.getEntry("AprilTagY");
        this.delay = fromPython.getEntry("Delay");
        this.systemTime = fromPython.getEntry("SystemTime");
    
        this.winch32Position_in = smartDashboard.getEntry("Winch 32 (in.)");
        this.winch33Position_in = smartDashboard.getEntry("Winch 33 (in.)");
        this.winch32AtTop = smartDashboard.getEntry("Winch 32 @ Top");
        this.winch33AtTop = smartDashboard.getEntry("Winch 33 @ Top");
        this.winch32PositionError = smartDashboard.getEntry("Winch 32 Error");
        this.winch33PositionError = smartDashboard.getEntry("Winch 33 Error");
        this.x = smartDashboard.getEntry("X");
        this.y = smartDashboard.getEntry("Y");
        this.gyroEnabled= smartDashboard.getEntry("Gyro Enabled");
        this.vacSensor = smartDashboard.getEntry("Vacuum Sensor");
        this.feederSpeed = smartDashboard.getEntry("Feeder Speed");
        this.flywheelSpeed = smartDashboard.getEntry("Flywheel Speed");
        this.cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    }

    public void setWinchPositions(
            double winch32Position_in, 
            boolean winch32AtTop,
            double winch33Position_in,
            boolean winch33AtTop) {
        this.winch32Position_in.setDouble(winch32Position_in);
        this.winch32AtTop.setBoolean(winch32AtTop);
        this.winch33Position_in.setDouble(winch33Position_in);
        this.winch33AtTop.setBoolean(winch33AtTop);
    }

    public void setWinchPositionErrors(
            boolean winch32PositionError,
            boolean winch33PositionError) {
        this.winch32PositionError.setBoolean(winch32PositionError);
        this.winch33PositionError.setBoolean(winch33PositionError);
    }

    public void setVacuumSensor(double value) {
        this.vacSensor.setDouble(value);
    }

    public void setXY(Pose2d pose) {
        this.x.setDouble(pose.getX());
        this.y.setDouble(pose.getY());
    }

    public void setGyroEnabled(boolean enabled) {
        this.gyroEnabled.setBoolean(enabled);
    }

    public void setFeederSpeed(double speed) {
        this.feederSpeed.setDouble(speed);
    }

    public void setFlywheelSpeed(double speed) {
        this.flywheelSpeed.setDouble(speed);
    }

    public void setCameraLimelight() {
        this.cameraSelection.setString("gloworm-output");
    }

    public void setCameraJump() {
        this.cameraSelection.setString("Jump");
    }
    
    public boolean getAprilTagPresent() { // true if apriltag(s) detected.
        return this.aprilTagPresent.getBoolean(false);
    }

    public Double[] getAprilTags() {
        Double[] list = {aprilTagX.getDouble(0.0), aprilTagY.getDouble(0.0)};
        return list;

    }
    public double getDelay() {
        // System.out.println("RIO" + (Instant.now().toEpochMilli() ));
        // System.out.println(delay.getDouble(0));
        return Instant.now().toEpochMilli() - systemTime.getDouble(-20000) + delay.getDouble(0);
    }

    public void initializeSwerveCommands(
            GyroEnableCommand gyroEnableCommand,
            GyroDisableCommand gyroDisableCommand) {
        SmartDashboard.putData(gyroEnableCommand);
        SmartDashboard.putData(gyroDisableCommand);
    }

    public void initializeVacuumAndWinchCommands(
            VacuumRunCommand vacuumRunCommand,
            WinchAndVacuumCommand winchAndVacuumCommand) {
        SmartDashboard.putData(vacuumRunCommand);
        SmartDashboard.putData(winchAndVacuumCommand);
    }

    public void initializeGrapplerCommands(
            GrapplerLockCommand grapplerLockCommand,
            GrapplerUnlockCommand grapplerUnlockCommand
            
            ) {
        SmartDashboard.putData(grapplerLockCommand);
        SmartDashboard.putData(grapplerUnlockCommand);
    }
}