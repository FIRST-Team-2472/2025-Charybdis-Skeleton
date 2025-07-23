package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.RobotStatus;
import frc.robot.MotorPowerController;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.NewNewAccelLimiter;
import frc.robot.extras.SwerveModule;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.extras.VL53L4CD;

public class SwerveSubsystem extends SubsystemBase {

    VL53L4CD timeOfFlightSensor = new VL53L4CD(I2C.Port.kOnboard);

    private SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    // It's still going, oh my god.
    private SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private Pigeon2 gyro = new Pigeon2(SensorConstants.kPigeonID);
    // Sets the preliminary odometry. This gets refined by the PhotonVision class,
    // but this is the original.
    // private final SwerveDriveOdometry odometer = new
    // SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    // new Rotation2d(0), getModulePositions());

    private final SwerveDrivePoseEstimator robotPoseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getRotation2d(), getModulePositions(), new Pose2d(),
            VecBuilder.fill(0.5, 0.5, 0.1),
            VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE));
    
    private final SwerveDriveOdometry frontLeftWheelPosition = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getFrontLeftModulePositions(),
        new Pose2d()
    );
    private final SwerveDriveOdometry frontRightWheelPosition = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getFrontRightModulePositions(),
        new Pose2d()
    );
    private final SwerveDriveOdometry backLeftWheelPosition = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getBackLeftModulePositions(),
        new Pose2d()
    );
    private final SwerveDriveOdometry backRightWheelPosition = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getBackRightModulePositions(),
        new Pose2d()
    );
            
    private GenericEntry headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB;

    private NewNewAccelLimiter speedLimiter;
    public PIDController thetaController;

    public MotorPowerController speedPowerController, turningPowerController;

    double lastXDrive = 0;
    double lastYDrive = 0;
    /*
     * double distanceError = 0;
     * double xError = 0;
     * double yError = 0;
     */

    double xError, yError, xSpeed, ySpeed, distanceError, angleDifference, speed, velocityX, velocityY, lastYawValue;
    double frontLeftLowRefreshPosition, frontRightLowRefreshPosition, backLeftLowRefreshPosition, backRightLowRefreshPosition;
    double frontLeftHighRefreshPosition, frontRightHighRefreshPosition, backLeftHighRefreshPosition, backRightHighRefreshPosition;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        timeOfFlightSensor.init();
        timeOfFlightSensor.startRanging();

        speedLimiter = new NewNewAccelLimiter(TargetPosConstants.kForwardMaxAcceleration,
                TargetPosConstants.kBackwardMaxAcceleration);

        speedLimiter = new NewNewAccelLimiter(TargetPosConstants.kForwardMaxAcceleration,
                TargetPosConstants.kBackwardMaxAcceleration);

        speedPowerController = new MotorPowerController(0.1, 0.04, .005, 1, .2, 0, 1);
        turningPowerController = new MotorPowerController(0.15, 0.02, .7, 1, .1, 0, 1);

        // zeros heading after pigeon boots up)()
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public SwerveSubsystem(Pigeon2 gyro, // This constructor is used for testing
            SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight,
            GenericEntry headingShuffleBoard, GenericEntry odometerShuffleBoard, GenericEntry rollSB,
            GenericEntry pitchSB) {
        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.headingShuffleBoard = headingShuffleBoard;
        this.odometerShuffleBoard = odometerShuffleBoard;
        this.rollSB = rollSB;
        this.pitchSB = pitchSB;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return robotPoseEstimator;
    }

    // Just a quick method that zeros the IMU.
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public void zeroRobotHeading() {
        robotPoseEstimator.resetPosition(getRotation2d(), getModulePositions(),
                new Pose2d(robotPoseEstimator.getEstimatedPosition().getX(),
                        robotPoseEstimator.getEstimatedPosition().getY(), new Rotation2d()));
    }

    // Gets the yaw/heading of the robot. getting this right is very important for
    // swerve
    public double getHeading() {
        // imu is backwards, so it is multiplied by negative one
        return gyro.getYaw().getValueAsDouble();
    }

    // Gets the roll of the robot based on the IMU.
    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    // Gets the pitch of the robot based on what the IMU percieves.
    public double getPitch() {
        return -gyro.getPitch().getValueAsDouble();
    }

    // gets our current velocity relative to the x of the field
    public double getXSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vxMetersPerSecond;
    }

    /*
     * public ChassisSpeeds getChassisSpeedsRobotRelative() {
     * return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotation2d());
     * }
     */

    public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getRotation2d());
    }

    // gets our current velocity relative to the x of the robot (front/back)
    public double getXSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the y of the field
    public double getYSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vyMetersPerSecond;
    }

    // gets our current velocity relative to the y of the robot (left/right)
    public double getYSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vyMetersPerSecond;
    }

    // gets our current angular velocity
    public double getRotationalSpeed() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.omegaRadiansPerSecond;
    }

    // Gets our heading and translates it to Rotation2d
    // (all of swerve methods use Rotation2d)
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /*
     * public void zeroOdometry() {
     * odometer.resetPosition(new Rotation2d(0), getModulePositions(), new
     * Pose2d());
     * }
     */

    public void zeroPoseEstimator() {
        robotPoseEstimator.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d());
    }

    /*
     * public void setOdometry(Pose2d odometryPose) {
     * 
     * odometer.resetPosition(getRotation2d(), getModulePositions(), odometryPose);
     * }
     */

    public void setPoseEstimator(Pose2d Pose) {
        robotPoseEstimator.resetPosition(getRotation2d(), getModulePositions(), Pose);
    }

    // Gets our drive position aka where the odometer thinks we are
    /*
     * public Pose2d getPose() {
     * return odometer.getPoseMeters();
     * }
     */

    public Pose2d getPose() {
        return robotPoseEstimator.getEstimatedPosition();
    }

    /*
     * public Pose2d getFilteredPose(double odometryConfidence) {
     * return this.positionFilteringSubsystem.getFilteredBotPose(odometer,
     * odometryConfidence);
     * }
     * 
     * public void calibrateOdometry(double odometryConfidence) {
     * Pose2d filteredPose = this.calculateFilteredPose(odometryConfidence);
     * Pose2d odometryPose = this.getOdometryPose();
     * //System.out.println(filteredPose);
     * //System.out.println(this.getOdometryPose());
     * //System.out.println(filteredPose.minus(this.getOdometryPose()));
     * 
     * // Calculate difference between odometry pose and filtered pose
     * // and set the odometry offset to that difference
     * 
     * this.odometryOffset = new Pose2d(filteredPose.getX() - odometryPose.getX(),
     * filteredPose.getY() - odometryPose.getY(), new Rotation2d());
     * }
     * 
     * public void calibrateOdometry() {
     * odometer.resetPosition(getRotation2d(), getModulePositions(),
     * getFilteredPose(1.0));
     * }
     */

    public void initializeDriveToPointAndRotate(Pose2d targetPosition) {
        xError = targetPosition.getX() - getPose().getX();// getPose().getX() - targetPosition.getX();
        yError = targetPosition.getY() - getPose().getY();// getPose().getY() - targetPosition.getY();
        distanceError = Math.sqrt(xError * xError + yError * yError);
        // xPowerController.calculate(getPose().getX(), targetPosition.getX());
        // yPowerController.calculate(getPose().getY(), targetPosition.getY());
        Rotation2d angleDifference = robotPoseEstimator.getEstimatedPosition().getRotation()
                .minus(targetPosition.getRotation());
        turningPowerController.calculate(angleDifference.getRadians(), 0);
        speedLimiter.zeroSpeed();
        // yLimiter.setInitialSpeed(lastYDrive);
    }

    public void executeDriveToPointAndRotate(Pose2d targetPosition) {
        xError = targetPosition.getX() - getPose().getX();// getPose().getX() - targetPosition.getX();
        yError = targetPosition.getY() - getPose().getY();// getPose().getY() - targetPosition.getY();
        SmartDashboard.putNumber("calculated x error", xError);
        SmartDashboard.putNumber("calculated y error", yError);
        distanceError = Math.sqrt(xError * xError + yError * yError);
        double xSpeed = xError / (Math.abs(xError) > Math.abs(yError) ? Math.abs(xError) : Math.abs(yError));
        double ySpeed = yError / (Math.abs(xError) > Math.abs(yError) ? Math.abs(xError) : Math.abs(yError));
        double speed = speedPowerController.calculate(0, distanceError);
        SmartDashboard.putNumber("distance Error", distanceError);
        SmartDashboard.putNumber("target X", targetPosition.getX());
        SmartDashboard.putNumber("target Y", targetPosition.getY());
        speed = -speedLimiter.calculate(speed);
        SmartDashboard.putNumber("speed", speed);
        xSpeed *= speed;
        ySpeed *= speed;

        SmartDashboard.putNumber("X speed", xSpeed);
        SmartDashboard.putNumber("Y speed", ySpeed);
        // double xSpeed = xPowerController.calculate(getPose().getX(),
        // targetPosition.getX());
        // double ySpeed = yPowerController.calculate(getPose().getY(),
        // targetPosition.getY());

        // angleDifference is the error value for the Motor Power Controller
        Rotation2d angleDifference = robotPoseEstimator.getEstimatedPosition().getRotation()
                .minus(targetPosition.getRotation());
        double turningSpeed = -turningPowerController.calculate(angleDifference.getRadians(), 0);

        // turningSpeed *= TargetPosConstants.kMaxAngularSpeed;
        // turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians,
        // turningSpeed);

        // ySpeed = yLimiter.calculate(ySpeed);
        // turningSpeed = turningLimiter.calculate(turningSpeed);

        // xSpeed = xLimiter.calculate(xSpeed);
        // ySpeed = yLimiter.calculate(ySpeed);

        runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }

    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }
    public SwerveModulePosition[] getFrontLeftModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontLeft.getPosition(), frontLeft.getPosition(),
            frontLeft.getPosition() };
        return temp;
    }
    public SwerveModulePosition[] getFrontRightModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontRight.getPosition(), frontRight.getPosition(), frontRight.getPosition(),
            frontRight.getPosition() };
        return temp;
    }
    public SwerveModulePosition[] getBackLeftModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { backLeft.getPosition(), backLeft.getPosition(), backLeft.getPosition(),
            backLeft.getPosition() };
        return temp;
    }
    public SwerveModulePosition[] getBackRightModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { backRight.getPosition(), backRight.getPosition(), backRight.getPosition(),
            backRight.getPosition() };
        return temp;
    }

    /*
     * public void runModulesFieldRelative(double xSpeed, double ySpeed, double
     * turningSpeed) {
     * lastXDrive = xSpeed;
     * lastYDrive = ySpeed;
     * // Converts robot speeds to speeds relative to field
     * ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
     * xSpeed, ySpeed, turningSpeed, odometer.getPoseMeters().getRotation());
     * 
     * // Convert chassis speeds to individual module states
     * SwerveModuleState[] moduleStates =
     * DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
     * 
     * // Output each module states to wheels
     * setModuleStates(moduleStates);
     * }
     */

    public void runModulesFieldRelative(double xSpeed, double ySpeed, double turningSpeed) {
        lastXDrive = xSpeed;
        lastYDrive = ySpeed;
        // Converts robot speeds to speeds relative to field
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, robotPoseEstimator.getEstimatedPosition().getRotation());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void runModulesRobotRelative(ChassisSpeeds chassisSpeeds) {
        // Converts robot speeds to speeds relative to field
        this.chassisSpeeds = chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void stopModules() {
        // Stops all of the modules. Use in emergencies.
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public boolean isExactlyInPosition(Pose2d targetPosition) {
        return isAtPoint(targetPosition.getTranslation()) && isAtAngle(targetPosition.getRotation());
    }

    public boolean isNearlyInPosition(Pose2d targetPosition) {
        return isNearPoint(targetPosition.getTranslation()) && isNearAngle(targetPosition.getRotation());
    }

    public boolean isAtPoint(Translation2d targetDrivePos) {
        SmartDashboard.putNumber("translation Error", getPose().getTranslation().getDistance(targetDrivePos));
        boolean isAtPose = getPose().getTranslation()
                .getDistance(targetDrivePos) <= TargetPosConstants.kAcceptableDistanceError; //

        SmartDashboard.putBoolean("isAtPose", isAtPose);
        return isAtPose;
    }

    public boolean isNearPoint(Translation2d targetDrivePos) {
        SmartDashboard.putNumber("translation Error", getPose().getTranslation().getDistance(targetDrivePos));
        boolean isNearPose = getPose().getTranslation()
                .getDistance(targetDrivePos) <= TargetPosConstants.kAcceptableDistanceError * 2.5; //

        SmartDashboard.putBoolean("isNearPose", isNearPose);
        return isNearPose;
    }

    public boolean isAtAngle(Rotation2d angle) {
        SmartDashboard.putNumber("angle error",
                Math.abs(robotPoseEstimator.getEstimatedPosition().getRotation().minus(angle).getDegrees()));
        boolean isAtAngle = Math.abs(robotPoseEstimator.getEstimatedPosition().getRotation().minus(angle)
                .getDegrees()) <= TargetPosConstants.kAcceptableAngleError;

        SmartDashboard.putBoolean("isAtAngle", isAtAngle);
        return isAtAngle;
    }

    public boolean isNearAngle(Rotation2d angle) {
        SmartDashboard.putNumber("angle error",
                Math.abs(robotPoseEstimator.getEstimatedPosition().getRotation().minus(angle).getDegrees()));
        boolean isNearAngle = Math.abs(robotPoseEstimator.getEstimatedPosition().getRotation().minus(angle)
                .getDegrees()) <= TargetPosConstants.kAcceptableAngleError;

        SmartDashboard.putBoolean("isNearAngle", isNearAngle);
        return isNearAngle;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        logSwerveDesiredStates(desiredStates);
        // if their speed is larger then the physical max speed, it reduces all speeds
        // until they are smaller than physical max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedPercent);
        // sets the modules to desired states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        // Resets all of the encoders on the robot.
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void periodic() {

        double temp = timeOfFlightSensor.measure().distanceMillimeters;
        RobotStatus.kTimeOfFlightDistance = temp;
        SmartDashboard.putNumber("distance sensor", temp);
        
        // this method comes from the subsystem class we inherited. Runs constantly
        // while robot is on

        // changes heading and module states into an x,y coordinate.
        // Updates everything based on the new information it gathers.
        frontLeftWheelPosition.update(getRotation2d(), getFrontLeftModulePositions());
        frontRightWheelPosition.update(getRotation2d(), getFrontRightModulePositions());
        backLeftWheelPosition.update(getRotation2d(), getBackLeftModulePositions());
        backRightWheelPosition.update(getRotation2d(), getBackRightModulePositions());
        robotPoseEstimator.updateWithTime(MathSharedStore.getTimestamp(), getRotation2d(), getModulePositions());

        RobotStatus.pigeonPitch = getPitch();
        RobotStatus.pigeonRoll = getRoll();
        RobotStatus.pigeonYaw = getHeading();

        SmartDashboard.putNumber("frontLeft turning Encoder",
                frontLeft.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("frontRight turning Encoder",
                frontRight.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackLeft turning Encoder", backLeft.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackRight turning Encoder",
                backRight.absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        
        SmartDashboard.putNumber("BackLeftDrive Velocity", backLeft.getDrivePosition() - backLeftLowRefreshPosition);
        SmartDashboard.putNumber("BackRightDrive Velocity", backRight.getDrivePosition() - backRightLowRefreshPosition);
        SmartDashboard.putNumber("FrontLeftDrive Velocity", frontLeft.getDrivePosition() - frontLeftLowRefreshPosition);
        SmartDashboard.putNumber("FrontRightDrive Velocity", frontRight.getDrivePosition() - frontRightLowRefreshPosition);

        SmartDashboard.putNumber("BackLeftDrive high Velocity", backLeft.getDrivePositionTwo() - backLeftHighRefreshPosition);
        SmartDashboard.putNumber("BackRightDrive high Velocity", backRight.getDrivePositionTwo() - backRightHighRefreshPosition);
        SmartDashboard.putNumber("FrontLeftDrive high Velocity", frontLeft.getDrivePositionTwo() - frontLeftHighRefreshPosition);
        SmartDashboard.putNumber("FrontRightDrive high Velocity", frontRight.getDrivePositionTwo() - frontRightHighRefreshPosition);

        SmartDashboard.putNumber("frontLeft X", frontLeftWheelPosition.getPoseMeters().getX());
        SmartDashboard.putNumber("frontLeft Y", frontLeftWheelPosition.getPoseMeters().getY());
        SmartDashboard.putNumber("frontRight X", frontRightWheelPosition.getPoseMeters().getX());
        SmartDashboard.putNumber("frontRight Y", frontRightWheelPosition.getPoseMeters().getY());
        SmartDashboard.putNumber("backLeft X", backLeftWheelPosition.getPoseMeters().getX());
        SmartDashboard.putNumber("backLeft Y", backLeftWheelPosition.getPoseMeters().getY());
        SmartDashboard.putNumber("backRight X", backRightWheelPosition.getPoseMeters().getX());
        SmartDashboard.putNumber("backRight Y", backRightWheelPosition.getPoseMeters().getY());

        frontLeftLowRefreshPosition = frontLeft.getDrivePosition();
        frontRightLowRefreshPosition = frontRight.getDrivePosition();
        backLeftLowRefreshPosition = backLeft.getDrivePosition();
        backRightLowRefreshPosition = backRight.getDrivePosition();

        frontLeftHighRefreshPosition = frontLeft.getDrivePositionTwo();
        frontRightHighRefreshPosition = frontRight.getDrivePositionTwo();
        backLeftHighRefreshPosition = backLeft.getDrivePositionTwo();
        backRightHighRefreshPosition = backRight.getDrivePositionTwo();

        SmartDashboard.putNumber("BackLeftDriveEncoder", backLeft.getDrivePosition());
        SmartDashboard.putNumber("BackRightDriveEncoder", backRight.getDrivePosition());
        SmartDashboard.putNumber("FrontLeftDriveEncoder", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FrontRightDriveEncoder", frontRight.getDrivePosition());

        SmartDashboard.putNumber("front left drive encoder high refresh", frontLeft.getDrivePositionTwo());
        SmartDashboard.putNumber("front right drive encoder high refresh", frontRight.getDrivePositionTwo());
        SmartDashboard.putNumber("back left drive encoder high refresh", backLeft.getDrivePositionTwo());
        SmartDashboard.putNumber("back right drive encoder high refresh", backRight.getDrivePositionTwo());
        

        SmartDashboard.putNumber("read frontLeft Encoder", frontLeft.getAbsolutePosition());
        SmartDashboard.putNumber("read frontRight Encoder", frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("read BackLeft Encoder", backLeft.getAbsolutePosition());
        SmartDashboard.putNumber("read BackRight Encoder", backRight.getAbsolutePosition());

        // SmartDashboard.putNumber("odometerX", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("EstimatedX", robotPoseEstimator.getEstimatedPosition().getX());
        // SmartDashboard.putNumber("odometerY", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("EstimatedY", robotPoseEstimator.getEstimatedPosition().getY());
        // SmartDashboard.putNumberArray("odometer", new double[] {
        // odometer.getPoseMeters().getX(),
        // odometer.getPoseMeters().getY(),
        // odometer.getPoseMeters().getRotation().getRadians() });
        SmartDashboard.putNumberArray("EstimatedPosition",
                new double[] { robotPoseEstimator.getEstimatedPosition().getX(),
                        robotPoseEstimator.getEstimatedPosition().getY(),
                        robotPoseEstimator.getEstimatedPosition().getRotation().getRadians() });
        // SmartDashboard.putNumber("odometerAngle",
        // odometer.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("EstimatedAngle",
                robotPoseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro Yaw", gyro.getYaw().getValueAsDouble());

        SmartDashboard.putNumber("frontLeftCurrent", frontLeft.getCurrent());
        SmartDashboard.putNumber("backLeftCurrent", backLeft.getCurrent());
        SmartDashboard.putNumber("frontRightCurrent", frontRight.getCurrent());
        SmartDashboard.putNumber("backRightCurrent", backRight.getCurrent());

        logSwerveStates();
        logOdometry();
        logPigeonState();
    }

    // Send the swerve modules' encoder positions to Advantage Kit
    public void logSwerveStates() {

        Logger.recordOutput("SwerveState", new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    // Send Swerve Desired Rotation and speed to Advantage Kit
    public void logSwerveDesiredStates(SwerveModuleState[] desiredStates) {
        Logger.recordOutput("DesiredSwerveState", desiredStates);
    }

    // Send Pigeon Rotation to Advantage Kit (useful for seeing robot rotation)
    public void logPigeonState() {
        Logger.recordOutput("PigeonGyro", gyro.getRotation2d());
    }

    // Send Odometry Position on field to Advantage Kit
    public void logOdometry() {
        Logger.recordOutput("Odometry/Location", getPose());
    }
}