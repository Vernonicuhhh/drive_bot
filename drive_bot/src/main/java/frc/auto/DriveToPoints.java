package frc.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.subsystems.Drive;
import frc.subsystems.RobotTracker;

public class DriveToPoints extends AutoCommand {

	private ArrayList<Translation2d> points;
	private double speed;
	private boolean isReversed;
	private Rotation2d endAngle;

	public DriveToPoints(double speed, boolean isReversed, Rotation2d endingAngle, Translation2d... points) {
		this.points = new ArrayList<Translation2d>();
		this.speed = speed;
		this.isReversed = isReversed;
		this.endAngle = endingAngle;
		setBlocking(true);
		for (Translation2d point : points) {
			this.points.add(point);
		}
	}

	@Override
	public void start() {
		System.out.println("Drive To Points");
		TrajectoryConfig config = new TrajectoryConfig(speed, Constants.DriveConstants.MAX_ACCEL_AUTO);
		config.addConstraint(Constants.DriveConstants.VOLTAGE_CONSTRAINT);
		config.setReversed(isReversed);
		Translation2d end = points.get(points.size()-1);
		points.remove(end);
		Trajectory drivePath = TrajectoryGenerator.generateTrajectory(
			RobotTracker.getInstance().getOdometry(),
			points,
			new Pose2d(end, endAngle),
			config
			);
		Drive.getInstance().setAutoPath(drivePath);
	}
	

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isFinished();
	}

}