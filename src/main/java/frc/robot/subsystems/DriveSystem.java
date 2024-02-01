// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSystem extends SubsystemBase {
    // absolute encoder offsets (0-1 | 0-360)
    // backleft (ID 12): -0.625732421875 | -225.263671875
    // backright (ID 9): -0.5673828125 | -204.2578125
    // frontleft (ID 3): -0.422607421875 | -152.138671875
    // frontright (ID 6): -0.292236328125 | -105.205078125

    private final SwerveDrive swerve;

    private double maxSpeed = Units.feetToMeters(DriveConstants.MAX_SPEED);

    /** Creates a new Drive. */
    public DriveSystem(File directory) {
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER),
                DriveConstants.DRIVE_GEAR_RATIO, DriveConstants.DRIVE_PULSE_PER_ROTATION);
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(DriveConstants.ANGLE_GEAR_RATIO,
                DriveConstants.ANGLE_PULSE_PER_ROTATION);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerve = new SwerveParser(directory)
                    .createSwerveDrive(
                            Units.feetToMeters(DriveConstants.MAX_SPEED), driveConversionFactor, angleConversionFactor);
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }

        swerve.setHeadingCorrection(false);

        swerve.pushOffsetsToControllers();

        setupPathPlanner();
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */

    public DriveSystem(SwerveDriveConfiguration driveConf,
            SwerveControllerConfiguration controllerConf) {
        swerve = new SwerveDrive(driveConf, controllerConf, maxSpeed);
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                        // Constants class
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Translation PID constants
                        new PIDConstants(swerve.swerveController.config.headingPIDF.p,
                                swerve.swerveController.config.headingPIDF.i,
                                swerve.swerveController.config.headingPIDF.d),
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */

    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return AutoBuilder.followPath(path);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerve.getMaximumVelocity(), 4.0,
                swerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */

    /*
     * public Command driveCommand(DoubleSupplier translationX, DoubleSupplier
     * translationY, DoubleSupplier headingX,
     * DoubleSupplier headingY) {
     * // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
     * // correction for this kind of control.
     * return run(() -> {
     * double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll
     * out
     * double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll
     * out
     * // Make the robot move
     * driveFieldOriented(swerve.swerveController.getTargetSpeeds(xInput, yInput,
     * headingX.getAsDouble(),
     * headingY.getAsDouble(),
     * swerve.getOdometryHeading().getRadians(),
     * swerve.getMaximumVelocity()));
     * });
     * }
     */

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */

    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {
            // Make the robot move
            driveFieldOriented(swerve.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerve.getOdometryHeading().getRadians(),
                    swerve.getMaximumVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerve.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) *
                    swerve.getMaximumVelocity(),
                    Math.pow(translationY.getAsDouble(), 3) * swerve.getMaximumVelocity()),
                    Math.pow(angularRotationX.getAsDouble(), 3) *
                            swerve.getMaximumAngularVelocity(),
                    true,
                    false);
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerve.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerve.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */

    public void drive(ChassisSpeeds velocity) {
        swerve.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */

    public SwerveDriveKinematics getKinematics() {
        return swerve.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerve.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */

    public Pose2d getPose() {
        return swerve.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerve.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */

    public void postTrajectory(Trajectory trajectory) {
        swerve.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */

    public void zeroGyro() {
        swerve.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */

    public void setMotorBrake(boolean brake) {
        swerve.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */

    /*
     * public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double
     * headingX, double headingY) {
     * xInput = Math.pow(xInput, 3);
     * yInput = Math.pow(yInput, 3);
     * return swerve.swerveController.getTargetSpeeds(xInput,
     * yInput,
     * headingX,
     * headingY,
     * getHeading().getRadians(),
     * maxSpeed);
     * }
     */

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerve.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maxSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */

    public ChassisSpeeds getFieldVelocity() {
        return swerve.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */

    public SwerveController getSwerveController() {
        return swerve.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerve.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */

    public void lock() {
        swerve.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */

    public Rotation2d getPitch() {
        return swerve.getPitch();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */

    public void addFakeVisionReading() {
        swerve.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

}
