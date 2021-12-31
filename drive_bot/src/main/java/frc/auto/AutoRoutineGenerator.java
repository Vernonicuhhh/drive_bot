package frc.auto;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.robot.Constants;

public class AutoRoutineGenerator {
    

//TODO: make autopath and test



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
