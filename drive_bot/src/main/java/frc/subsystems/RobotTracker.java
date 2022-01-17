package frc.subsystems;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;
import frc.util.multithreading.Threaded;

public class RobotTracker extends Threaded {

    private static RobotTracker instance;
    private Drive drive = Drive.getInstance();

    private DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(drive.getRotation2d()); // can I just use the getRotation2d or do I have to create a new rotation 2d object?
    private Pose2d poseMeters;
    private Pose2d estimatedPose;
    Field2d field = new Field2d();

    //TODO: Tune Gains for stdDev matrices
    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, .001, 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(.05, .05, Units.degreesToRadians(1.25));
    Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
        drive.getRotation2d(),
        new Pose2d(),
        stateStdDevs,
        localMeasurementStdDevs,
        visionMeasurementStdDevs,
        .02
        );

    @Override
    public void update(){
        Rotation2d heading = drive.getRotation2d();
        double leftDist = drive.getLeftDistance();
        double rightDist = drive.getRightDistance();

        synchronized(this){
            differentialDriveOdometry.update(heading, leftDist, rightDist);
            poseEstimator.update(drive.getRotation2d(), drive.getWheelSpeeds(), drive.getLeftDistance(), drive.getRightDistance());
            poseMeters = differentialDriveOdometry.getPoseMeters();
            estimatedPose = poseEstimator.getEstimatedPosition();
            field.setRobotPose(estimatedPose);
            


			SmartDashboard.putNumber("PoseX", differentialDriveOdometry.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("PoseY", differentialDriveOdometry.getPoseMeters().getTranslation().getY());
            SmartDashboard.putNumber("PoseR", differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());
            SmartDashboard.putData("minimap",field);
			SmartDashboard.putNumber("Estimated PoseX", poseEstimator.getEstimatedPosition().getTranslation().getX());
			SmartDashboard.putNumber("Estimated PoseY", poseEstimator.getEstimatedPosition().getTranslation().getY());
            SmartDashboard.putNumber("Estimated PoseR", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        }
    }

    public static RobotTracker getInstance(){
        if(instance == null)
            instance = new RobotTracker();
        return instance;
    }

    public synchronized void setOdometry(Pose2d pose){
        drive.zeroSensors();
        differentialDriveOdometry.resetPosition(pose, pose.getRotation());
        poseMeters = differentialDriveOdometry.getPoseMeters();
        poseEstimator.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
    }

    public synchronized Pose2d getOdometry(){
        return poseMeters;
    }

    public synchronized Pose2d getEstimatedOdometry(){
        return estimatedPose;
    }
    
    public synchronized void resetOdometry(){
        drive.zeroSensors();
        differentialDriveOdometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
        poseMeters = differentialDriveOdometry.getPoseMeters();
        poseEstimator.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
    }
}
