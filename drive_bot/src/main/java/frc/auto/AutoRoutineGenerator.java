
package frc.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.robot.Constants;
import frc.subsystems.RobotTracker;

public class AutoRoutineGenerator {
    
	private static AutoRoutine initialDrive;
	//TODO: add auto chooser options during season to quickly switch between auto routines

	public static AutoRoutine simpleLineTest(){
		TrajectoryConfig config = createConfig(.5, .5, false);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			new Pose2d(2,0,Rotation2d.fromDegrees(0))
		),
		config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true));
		return initialDrive;
	}

    private static TrajectoryConfig createConfig(double velocity, double accel, boolean reversed) {
		TrajectoryConfig config = new TrajectoryConfig(velocity, accel);
		config.addConstraint(Constants.DriveConstants.VOLTAGE_CONSTRAINT);
		config.setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);
		DifferentialDriveKinematicsConstraint driveConstraint = new DifferentialDriveKinematicsConstraint
		(
			Constants.DriveConstants.DRIVE_KINEMATICS,
			Constants.DriveConstants.MAX_SPEED_AUTO
		);
		config.addConstraint(driveConstraint);
		config.setReversed(reversed);
		return config;
	}

    private static TrajectoryConfig createConfig(double velocity, double accel, double endVel, boolean reversed) {
		TrajectoryConfig config = new TrajectoryConfig(velocity, accel);
		config.addConstraint(Constants.DriveConstants.VOLTAGE_CONSTRAINT);
		config.setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);
		DifferentialDriveKinematicsConstraint driveConstraint = new DifferentialDriveKinematicsConstraint
		(
			Constants.DriveConstants.DRIVE_KINEMATICS,
			Constants.DriveConstants.MAX_SPEED_AUTO
		);
		config.addConstraint(driveConstraint);
		config.setReversed(reversed);
		config.setEndVelocity(endVel);
		return config;
	}
}