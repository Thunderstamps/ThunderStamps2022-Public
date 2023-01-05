package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controllers.IXboxController;
import frc.controllers.PovService;
import frc.robot.NetworkTableComms;
import frc.robot.Robot;
import frc.robot.mode.RobotMode;
import frc.robot.vision.LimelightVision;

public class SwerveSubsystem extends SubsystemBase{

    private static final double ROTATION_P = 7.0;

    private final IXboxController xbox;
    private final SteeringController steeringController1, steeringController2, steeringController3, steeringController4;
    private final DriveController driveController1, driveController2, driveController3, driveController4;
    private final SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4;
    private final RobotOrientedSwerve robotOrientedSwerve;
    private final FieldOrientedSwerve fieldOrientedSwerve;
    private final IGyro gyro;
    private final RobotMode mode;
    private final LimelightVision limelightVision;
    private double targetFieldOrientation_rad;  
    private final PovService povService;
    private final NetworkTableComms nt;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d pose = new Pose2d();
    private boolean homed = false;
    
    public SwerveSubsystem(
        IXboxController xbox,
        IGyro gyro,
        RobotMode mode,
        LimelightVision limelightVision,
        NetworkTableComms nt) {
        this.xbox = xbox;
        this.gyro = gyro;
        this.mode = mode;
        this.nt = nt;
        this.limelightVision = limelightVision;
        this.povService = xbox.getPovService();
        steeringController1 = new SteeringController(11, 19, 3064);
        steeringController2 = new SteeringController(12, 20, 90);
        steeringController3 = new SteeringController(13, 21, 932);
        steeringController4 = new SteeringController(14, 22, 996);

        driveController1 = new DriveController(15,6.12);
        driveController2 = new DriveController(16,6.12);
        driveController3 = new DriveController(17,6.12);
        driveController4 = new DriveController(18,6.12);

        swerveModule1 = new SwerveModule(steeringController1, driveController1, Vector2D.FromXY(13, -11), 0);
        swerveModule2 = new SwerveModule(steeringController2, driveController2, Vector2D.FromXY(-13, -11), 0);
        swerveModule3 = new SwerveModule(steeringController3, driveController3, Vector2D.FromXY(-13, 11), 0);
        swerveModule4 = new SwerveModule(steeringController4, driveController4, Vector2D.FromXY(13, 11), 0);

        ArrayList<ISwerveModule> swerveModules = new ArrayList<ISwerveModule>();

        swerveModules.add(swerveModule1);
        swerveModules.add(swerveModule2);
        swerveModules.add(swerveModule3);
        swerveModules.add(swerveModule4);

        robotOrientedSwerve = new RobotOrientedSwerve(swerveModules);
        
        fieldOrientedSwerve = new FieldOrientedSwerve(
            robotOrientedSwerve, 
            gyro, 
            130,
            Robot.MAX_ACCELERATION_TELEOP_G * 386, 
            Robot.MAX_ROTATION_RATE_RAD_S, 
            Robot.SCAN_TIME_S, 
            ROTATION_P);
        
        swerveDriveKinematics = new SwerveDriveKinematics(
            swerveModule1.getTranslation2d(),
            swerveModule2.getTranslation2d(),
            swerveModule3.getTranslation2d(),
            swerveModule4.getTranslation2d()
        );
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            new Rotation2d(), // gyro angle
            new Pose2d(0.0, 0.0, new Rotation2d()),
            swerveDriveKinematics, 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01) // Global measurement standard deviations. X, Y, and theta.
            );
    }

    public void RunRobotPeriodic() {

        if(!homed) {
            steeringController1.tryHoming();
            steeringController2.tryHoming();
            steeringController3.tryHoming();
            steeringController4.tryHoming();

            if(steeringController1.isHomed() && steeringController2.isHomed() && steeringController3.isHomed() && steeringController4.isHomed()) {
                this.homed = true;
            }
        }

        if(this.homed) {
            this.runSwerveOdometry();
        }
    }

    private void execute(
            Vector2D fieldOrientedTranslationCommand_in_s_rad,
            Vector2D robotOrientedTranslationCommand_in_s_rad,
            double robotOrientedRotationCommand_s_rad) {

        var anyManualCommand = false;

        if (robotOrientedRotationCommand_s_rad !=0.0) {
            var diff_rad = robotOrientedRotationCommand_s_rad * Robot.SCAN_TIME_S;
            this.targetFieldOrientation_rad += diff_rad;
            anyManualCommand = true;
        }

        var pov = this.povService.getPOV();

        if(this.mode.GetJumpCamera()) {
            if (xbox.getLeftBumperPressed()) {
                this.targetFieldOrientation_rad = this.gyro.getFieldOrientation_rad() + Math.toRadians(2.5);
            }
            if (xbox.getRightBumperPressed()) {
                this.targetFieldOrientation_rad = this.gyro.getFieldOrientation_rad() - Math.toRadians(2.5);
            }
        }
        else {
            if (xbox.getLeftBumperPressed()) {
                int prevPov = povService.getPrevPOV();
                if (prevPov != PovService.POV_NONE) {
                    this.targetFieldOrientation_rad = povService.getSnapAngleFromPovRadians(prevPov);
                    anyManualCommand = true;
                }
            }
    
            if (xbox.getRightBumperPressed()) {
                int nextPov = povService.getNextPOV();
                if (nextPov != PovService.POV_NONE) {
                    this.targetFieldOrientation_rad = povService.getSnapAngleFromPovRadians(nextPov);
                    anyManualCommand = true;
                }
            }
        }

        if (pov != PovService.POV_NONE) {
            this.targetFieldOrientation_rad = this.povService.getSnapAngleFromPovRadians(pov);
            anyManualCommand = true;
        }

        // auto aim
        var aiming = this.mode.GetAiming();
        var targetData = this.limelightVision.GetTargetData();
        var turnToTarget = aiming && targetData.ValidTargetData() && !anyManualCommand;
        if(turnToTarget) {
            var heading_rad = this.gyro.getFieldOrientation_rad();
            var target_rad = Math.toRadians(targetData.X_deg());
            this.targetFieldOrientation_rad = heading_rad - target_rad;
        }

        this.fieldOrientedSwerve.execute(
            fieldOrientedTranslationCommand_in_s_rad, 
            targetFieldOrientation_rad,
            robotOrientedTranslationCommand_in_s_rad, 
            robotOrientedRotationCommand_s_rad);
    }
    
    private Vector2D getRobotOrientedTranslationCommand_in_s() {
        var robotOrientedTranslation = getxboxControllerLeftStick();
        robotOrientedTranslation.squareMagnitude();
        double newMagnitude = robotOrientedTranslation.getMagnitude() * Robot.MAX_SPEED_IN_SEC;
        robotOrientedTranslation.setMagnitude(newMagnitude);
        if (robotOrientedTranslation.getMagnitude() < 0.5) {
            robotOrientedTranslation.setMagnitude(0.0);
        }
        return robotOrientedTranslation;
    }


    private Vector2D getFieldOrientedTranslationCommand_in_s() {
        Vector2D input;
        double maxSpeed_inSec;
        input = getxboxControllerRightStick();
        maxSpeed_inSec = Robot.MAX_SPEED_IN_SEC;

        input.squareMagnitude();
        var newMagnitude = input.getMagnitude() * maxSpeed_inSec;
        Vector2D fieldOrientedTranslation = Vector2D.FromPolar(newMagnitude, input.getAngleRadians());
        if(fieldOrientedTranslation.getMagnitude() < 0.2) {
            fieldOrientedTranslation.setMagnitude(0.0);
        }

        return fieldOrientedTranslation;

    }
    private double getRobotOrientedRotationCommand_rad_s() {
        var leftTrigger = xbox.getLeftTriggerAxis();
        var rightTrigger = xbox.getRightTriggerAxis();
        var triggerMagnitude = (leftTrigger - rightTrigger);
        var triggerMagnitudeSquared = triggerMagnitude * Math.abs(triggerMagnitude);
        return triggerMagnitudeSquared *Robot.MAX_ROTATION_RATE_RAD_S * 0.10;
    }
    
    private Vector2D getxboxControllerRightStick() {
        var x = xbox.getRightX();
        if(Math.abs(x) < 0.05) {
            x = 0.0;
        }
        var y = xbox.getRightY();
        if(Math.abs(y) < 0.05) {
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
        

    }
    private Vector2D getxboxControllerLeftStick() {
        var x = xbox.getLeftX();
        if(Math.abs(x) < 0.05) {
            x = 0.0;
        }
        var y = xbox.getLeftY();
        if(Math.abs(y) < 0.05) {
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
        

    }

    public void executeOperatorControl() {
        if(this.gyro.getEnabled()) {
            this.runSwerveFieldOriented();
        }
        else {
            this.runSwerveRobotOriented();
        }
    }
    private void runSwerveFieldOriented() {
        var fieldOrientedTranslationCommand_in_s_rad = getFieldOrientedTranslationCommand_in_s();
        var robotOrientedTranslationCommand_in_s_rad = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_in_s_rad = getRobotOrientedRotationCommand_rad_s();
        this.execute(
            fieldOrientedTranslationCommand_in_s_rad,
            robotOrientedTranslationCommand_in_s_rad,
            robotOrientedRotationCommand_in_s_rad
        );
    }
    
    private void runSwerveRobotOriented() {
        var robotOrientedTranslationCommand_in_s = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_rad_s = getRobotOrientedRotationCommand_rad_s();
    
        robotOrientedSwerve.execute(
          robotOrientedTranslationCommand_in_s, 
          robotOrientedRotationCommand_rad_s);
    }

    public void addPoseFromCamera(Pose2d pose_meters) {
        swerveDrivePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (nt.getDelay() / 1000));
      }
    

    private void runSwerveOdometry() {
      
        var gyroAngle = new Rotation2d(this.gyro.getFieldOrientation_rad());
    
        this.pose = 
          this.swerveDrivePoseEstimator.update(gyroAngle, 
          swerveModule1.getSwerveModuleState(), 
          swerveModule2.getSwerveModuleState(), 
          swerveModule3.getSwerveModuleState(), 
          swerveModule4.getSwerveModuleState());
        // var poseTranslation = this.pose.getTranslation();
        // System.out.printf("X: %.2f, Y: %.2f", poseTranslation.getX(), poseTranslation.getY());
        // System.out.println();
        if (nt.getAprilTagPresent()) {
          addPoseFromCamera(
              new Pose2d(
              nt.getAprilTags()[0],
              nt.getAprilTags()[1],
              gyroAngle)
              );
        }
      }

    private void printOdometry() {
        var poseTranslation = this.pose.getTranslation();
        System.out.printf("X: %.2f, Y: %.2f", poseTranslation.getX(), poseTranslation.getY());
        System.out.println();
    }


    public SwerveDriveKinematics getKinematics() {
      return this.swerveDriveKinematics;
    }

    public Pose2d getPose() {
      return this.pose;
    }
    
    public void zeroGyro() {
        gyro.resetOffsetToZero_rad();
        this.povService.zeroLastPOV();
        this.targetFieldOrientation_rad = 0.0;
    }

    public void enableGyro() {
        this.gyro.setEnabled(true);
    }

    public void disableGyro() {
        this.gyro.setEnabled(false);
    }

    public void executeAutonomousControl(
        Vector2D fieldOrientedTranslationCommand_in_s,
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {

        if(this.gyro.getEnabled()) {
            this.execute(
                fieldOrientedTranslationCommand_in_s, 
                robotOrientedTranslationCommand_in_s,
                robotOrientedRotationCommand_rad_s);
        }
        else {
            this.execute(Vector2D.FromXY(0, 0), Vector2D.FromXY(0, 0), 0.0);    
        }
    }
      
    public void executeAutonomousControlRobotOriented(
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {
        robotOrientedSwerve.execute(
            robotOrientedTranslationCommand_in_s, 
            robotOrientedRotationCommand_rad_s);
        runSwerveOdometry();
    }

    public void autonomousInit() {
        this.gyro.setEnabled(true);
        this.zeroGyro();
        this.fieldOrientedSwerve.setMaxAcceleration(Robot.MAX_ACCELERATION_AUTO_G*386.0);
        this.swerveDrivePoseEstimator.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d()), new Rotation2d());
    }

    public void teleopInit() {
        this.fieldOrientedSwerve.setMaxAcceleration(Robot.MAX_ACCELERATION_TELEOP_G*386.0);
        this.gyro.setEnabled(true);
        this.targetFieldOrientation_rad = this.gyro.getFieldOrientation_rad(); // prevents turning when you power up
    }
    
    public double getFieldOrientation_rad() {
        return this.gyro.getFieldOrientation_rad();
    }

    public void setGyroTo(double desiredOrientation_rad) {
        var currentAngle_rad = this.gyro.getFieldOrientation_rad();
        var adjustment_rad = currentAngle_rad - desiredOrientation_rad;
        this.gyro.adjustOffset_rad(adjustment_rad);
        this.targetFieldOrientation_rad = desiredOrientation_rad;
    }

    public void SetRobotFieldPosition(Pose2d pose, Rotation2d gyroAngle) {
      this.swerveDrivePoseEstimator.resetPosition(pose, gyroAngle);
    }
    public void setTargetFieldOrientation_rad(double targetFieldOrientation_rad) {
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;
    }
}
