package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.subsystems.Vision.Vision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private PIDController m_balancePID = new PIDController(Constants.Swerve.GAINS_BALANCE.kP, Constants.Swerve.GAINS_BALANCE.kI, Constants.Swerve.GAINS_BALANCE.kD);
    private PIDController m_thetaController = new PIDController(Constants.Swerve.GAINS_ANGLE_SNAP.kP, Constants.Swerve.GAINS_ANGLE_SNAP.kI, Constants.Swerve.GAINS_ANGLE_SNAP.kD);
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private SwerveRequest.ApplyChassisSpeeds driveCS = new SwerveRequest.ApplyChassisSpeeds().withIsOpenLoop(true); 
    Vision eyes;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        eyes = new Vision(this);
        m_thetaController.enableContinuousInput(-180, 180);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        eyes = new Vision(this);
        m_thetaController.enableContinuousInput(-180, 180);
    }

    public CommandBase applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return new RunCommand(()->{this.setControl(requestSupplier.get());}, this);
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (int i = 0; i < this.Modules.length; ++i) {
            SwerveModuleState state = desiredStates[i];
            this.Modules[i].apply(state, false);
        }
    }
    
    public Rotation2d getYaw(){
        return Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValueAsDouble());
    }

    public SwerveModulePosition[] getModulePositions(){
        return this.m_modulePositions;
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }

    public Pose2d getVisionPose(){
        return eyes.getCurrentPose();
    }

    public Pose2d getNearestGridPose(){
        //Check the cameras to see where we are
        double currentY = this.getVisionPose().getY();
        double closestY = 0;

        if(GamePiece.getGamePiece() == GamePieceType.Cube){
            //set the default starting position for Y
            if(DriverStation.getAlliance() == Alliance.Blue){
                 closestY = Constants.Swerve.BLUEcubeYcoord[0];
                for(int i = 1; i < Constants.Swerve.BLUEcubeYcoord.length; i++){
                    if(Math.abs(Constants.Swerve.BLUEcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        System.out.println("Closest Y Coord: " + closestY);
                        closestY = Constants.Swerve.BLUEcubeYcoord[i];
                    }
                }
                return new Pose2d(new Translation2d(1.84, closestY), Rotation2d.fromDegrees(180));
            }
            
            //Here we will write Red
            if(DriverStation.getAlliance() == Alliance.Red){
                closestY = Constants.Swerve.REDcubeYcoord[0];
                for(int i = 1; i< Constants.Swerve.REDcubeYcoord.length; i++){
                    if(Math.abs(Constants.Swerve.REDcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        closestY = Constants.Swerve.REDcubeYcoord[i];
                    }
                }
                return new Pose2d(new Translation2d(1.84, closestY), Rotation2d.fromDegrees(180));
            }           
        }

        if(GamePiece.getGamePiece() == GamePieceType.Cone){
            //set the default starting position for Y
            if(DriverStation.getAlliance() == Alliance.Blue){
                closestY = Constants.Swerve.BLUEconeYcoord[0];
               for(int i = 1; i < Constants.Swerve.BLUEconeYcoord.length; i++){
                   if(Math.abs(Constants.Swerve.BLUEconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                       System.out.println("Closest Y Coord: " + closestY);
                       closestY = Constants.Swerve.BLUEconeYcoord[i];
                   }
               }
               return new Pose2d(new Translation2d(1.84, closestY), Rotation2d.fromDegrees(180));
           }
           
           //Here we will write Red
           if(DriverStation.getAlliance() == Alliance.Red){
               closestY = Constants.Swerve.REDconeYcoord[0];
               for(int i = 1; i< Constants.Swerve.REDconeYcoord.length; i++){
                   if(Math.abs(Constants.Swerve.REDconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                       closestY = Constants.Swerve.REDconeYcoord[i];
                   }
               }
               return new Pose2d(new Translation2d(1.84, closestY), Rotation2d.fromDegrees(180));
           }     
        }

        //Defaults to the middle-ish of the grid but this shouldn't happen
        return new Pose2d(new Translation2d(2.3, 3.0), Rotation2d.fromDegrees(180));

    }

    public void stopDrive(){
        this.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds){
        this.setControl(driveCS.withSpeeds(speeds));
    }

    public void AutoBalance(double setpoint){
        m_balancePID.setTolerance(Constants.Swerve.BALANCE_TOLLERANCE);
        double pidOutput;
        pidOutput = MathUtil.clamp(m_balancePID.calculate(this.m_pigeon2.getRoll().getValueAsDouble(), 0), -1.0, 1.0);

        m_thetaController.setSetpoint(setpoint);
        
        double rotationVal = m_thetaController.calculate((MathUtil.inputModulus(this.getState().Pose.getRotation().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
        rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity * 0.075, Constants.Swerve.maxAngularVelocity * 0.075);

        if(Constants.tuningMode){
            SmartDashboard.putNumber("Balance PID", pidOutput);
        }

        this.setControl(drive.withVelocityX(-pidOutput).withVelocityY(0).withRotationalRate(rotationVal));
    }

    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory path1, boolean isFirstPath) {
        PIDController thetaController = new PIDController(2.5, 0, 0);
        PIDController xController = new PIDController(1.3, 0, 0);
        PIDController yController = new PIDController(1.3, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                    PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(path1, DriverStation.getAlliance());
                    this.seedFieldRelative(transformed.getInitialHolonomicPose());
                    eyes.resetPathPose(path1);
               }
             }),
             new PPSwerveControllerCommand(
                 path1, 
                 this::getPose, // Pose supplier
                 xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 yController, // Y controller (usually the same values as X controller)
                 thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setChassisSpeeds,  // Module states consumer
                 true, //Automatic mirroring
                 this // Requires this drive subsystem
             ) 
             .andThen(() -> stopDrive())
         );
     }

    @Override
    public void simulationPeriodic() {
        /* Assume  */
        updateSimState(0.02, 12);
    }
}
