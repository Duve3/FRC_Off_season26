// package frc.robot.commands;

// import static edu.wpi.first.units.Units.MetersPerSecond;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.*;
// import frc.robot.RobotContainer;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.LimelightSubsystem;

// public class AlignReef extends Command{

//     // Subsystems from RobotContainer
//     private LimelightSubsystem limelight;
//     private CommandSwerveDrivetrain drivetrain;
//     private RobotContainer robotContainer;

//     Timer timer = new Timer();

//     /* ----- PIDs ----- */
//     // 6,6,3
//     private PIDController pidX = new PIDController(6, 0, 0);
//     private PIDController pidY = new PIDController(6, 0, 0);
//     private PIDController pidRotate = new PIDController(3, 0, 0); 

//     // Creates a swerve request that specifies the robot to move FieldCentric
//     private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
//     .withDriveRequestType(DriveRequestType.Velocity); // Uses ClosedLoopVoltage for PID
//     // Creates a swerve request to stop all motion by setting velocities and rotational rate to 0
//     SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

//     // The Desired position to go to
//     private Pose2d targetPose;
//     // The speed to move to position
//     private final double speed = 1.0; //1.5
//     // The speed (rad/s) to rotate to position
//     private final double rotationSpeed = 0.75; //1.25
//     // The tolerance before stopping align (meters)
//     private final double positionTolerance = 0.01;
//     // The tolerance for yaw alignment (radians)
//     private final double yawTolerance = Math.PI / 32;
//     // Indicates if alignment uses PID Control
//     private final boolean usingPID = true;

//     // Indicates the Left or Right side of reef
//     ReefPos reefPos;
//     // X and Y Offset from the April Tag (Default: Reef)
//     private double offsetX = 0;
//     private double offsetY = 0;
//     // Targetted Tag ID
//     private int tID;
//     // Indicates if tag was detected
//     private boolean tagDetected;
//     // April Tags on the field
//     AprilTagFieldLayout aprilTagMap;
//     // Add a variable to store the first detected AprilTag ID
//     private Integer storedTagID = null;
//     // Flag to track if we've seen tag 10 or 21 first
//     //private boolean initiallyDetectedTag10 = false;
//     //private boolean initiallyDetectedTag21 = false;

//     private double calculateDistance(Pose2d pose1, Pose2d pose2){
//         double dx = pose1.getX() - pose2.getX();
//         double dy = pose1.getY() - pose2.getY();
//         return Math.sqrt(dx*dx + dy*dy);
//     }

//     // CONSTRUCTOR
//     public AlignReef(RobotContainer robotContainer, ReefPos reefPos){
//         this.robotContainer = robotContainer;
//         this.drivetrain = robotContainer.drivetrain;
//         //this.limelight = robotContainer.limelight;
//         this.reefPos = reefPos;

//         // -180 and 180 degrees are the same point, so its continuous
//         pidRotate.enableContinuousInput(-Math.PI, Math.PI);

//         // Added Logging
//         System.out.println("AlignReef command created for " + reefPos + " position");
//         Logger.recordOutput("Reefscape/AlignReef/ReefPosition", reefPos.toString());
//     }  
    
//     @Override
//     public void initialize(){
//         // Starts timer
//         timer.restart();
//         Logger.recordOutput("Reefscape/AlignReef/CommandStarted", true);
//         Logger.recordOutput("Reefscape/AlignReef/StartTime", timer.get());
        
//         double minDistance = Double.MAX_VALUE;
//         Pose2d robotPose = drivetrain.getState().Pose;
//         if (robotPose==null) {
//             tagDetected = false;
//             return;
//         }

//         for (int id : Constants.AprilTagMaps.aprilTagMap.keySet()) {
//             double[] aprilTagList = Constants.AprilTagMaps.aprilTagMap.get(id);
//             Pose2d aprilTagPose = new Pose2d(aprilTagList[0] * Constants.inToM, aprilTagList[1] * Constants.inToM, new Rotation2d(aprilTagList[3] * Math.PI / 180));
//             double distance = calculateDistance(robotPose, aprilTagPose);
//             if (distance < minDistance) {
//                 minDistance = distance;
//                 tID = id;
//                 targetPose = aprilTagPose;
//             }
//         }
    
//         // Check if a tag was found
//         if (minDistance == Double.MAX_VALUE) {
//             tagDetected = false;
//             System.out.println("Error: No AprilTag found.");
//             Logger.recordOutput("Reefscape/AlignReef/Error", "No AprilTag found.");
//             return;
//         }
    
//         // Store the first detected tag ID
//         if (storedTagID == null) {
//             storedTagID = tID;
//             Logger.recordOutput("Reefscape/AlignReef/StoredTagID", storedTagID);
//         }

//         // Log the closest tag ID and pose
//         Logger.recordOutput("Reefscape/AlignReef/TargetTagID", tID);
//         Logger.recordOutput("Reefscape/AlignReef/AprilTagPose", targetPose);

//         // Gets the tag ID that is being targeted
//         tID = limelight.getTid();
 
// /*         // Check if we initially see tag 10
//         if (tID == 10) {
//             initiallyDetectedTag10 = true;
//             System.out.println("Detected tag 10 - gonna keep this target if tag 11 appears");
//             Logger.recordOutput("Reefscape/AlignReef/InitiallyDetectedTag10", true);
//         } else if (tID == 21) {
//             initiallyDetectedTag21 = true;
//             System.out.println("Detected tag 21 - gonna keep this target if tag 20 appears");
//             Logger.recordOutput("Reefscape/AlignReef/InitiallyDetectedTag21", true);
//         } */

//         double[] aprilTagList = Constants.AprilTagMaps.aprilTagMap.get(tID);
//         // Checks if the tag exists within the list of all tags
//         if (aprilTagList == null) {
//             System.out.println("Error: Target pose array is null for Tag ID: " + tID);
//             Logger.recordOutput("Reefscape/AlignReef/Error", "Target pose array is null for Tag ID: " + tID);
//             // Command is useless, thus it will end
//             tagDetected = false;
//             return;
//         }
//         // Creates a Pose2D of the April Tag's position
//         Pose2d aprilTagPose = new Pose2d(aprilTagList[0] * Constants.inToM, aprilTagList[1] * Constants.inToM, new Rotation2d(aprilTagList[3] * Math.PI / 180));
//         // Tag is detected
//         tagDetected = true;
//         Logger.recordOutput("Reefscape/AlignReef/TagDetected", tagDetected);
//         Logger.recordOutput("Reefscape/AlignReef/AprilTagPose", aprilTagPose);
        
//         // Reef Offset Positions - log the chosen offsets
//         if (Constants.contains(new double[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}, tID)){
//             if (reefPos == ReefPos.LEFT){
//                 //System.out.println("left");
//                 offsetX = -0.41;
//                 offsetY = 0.13;
//             }
//             else if (reefPos == ReefPos.RIGHT){
//                 //System.out.println("right");
//                 offsetX = -0.41;
//                 offsetY = -0.2335;
//             }
//             Logger.recordOutput("Reefscape/AlignReef/OffsetX", offsetX);
//             Logger.recordOutput("Reefscape/AlignReef/OffsetY", offsetY);
//         }
        
//         // The target rotation of the robot is opposite of the april tag's rotation
//         double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
//         // AngleModulus normalizes the difference to always take the shortest path
//         targetRotation = MathUtil.angleModulus(targetRotation);
//         Logger.recordOutput("Reefscape/AlignReef/TargetRotation", targetRotation);

//         // Log offsets after rotation calculations
//         double newOffsetX = (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
//         double newOffsetY = (offsetX * Math.sin(targetRotation)) + ((offsetY * Math.cos(targetRotation)));
//         Logger.recordOutput("Reefscape/AlignReef/RotatedOffsetX", newOffsetX);
//         Logger.recordOutput("Reefscape/AlignReef/RotatedOffsetY", newOffsetY);
        
//         // Creates a Pose2d for the target position
//         targetPose = new Pose2d(aprilTagPose.getX() + newOffsetX, aprilTagPose.getY() + newOffsetY, new Rotation2d(targetRotation));
//         Logger.recordOutput("Reefscape/AlignReef/TargetPose", targetPose);

//         // Sets the destination to go to for the PID
//         pidX.setSetpoint(targetPose.getX());
//         pidY.setSetpoint(targetPose.getY());
//         // Converts to radians
//         pidRotate.setSetpoint(targetPose.getRotation().getRadians());
//         // Prints Target Pose
//         //System.out.println("Target Pose2d: " + targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getRotation().getDegrees());

//         // Sets Robot Max Speed for Alignment
//         robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; // remove 0.7
//     }

//     // Called every 20ms to perform actions of Command
//     @Override
//     public void execute(){
//         // If no tag was detected, then Command wont execute
//         if (!tagDetected){
//             Logger.recordOutput("Reefscape/AlignReef/ExecuteSkipped", true);
//             return;
//         }

// /*         // Get the current tag ID from limelight
//         int currentTagID = limelight.getTid();
        
//         // Maintain Tag
//         if (initiallyDetectedTag10 && currentTagID == 11) {
//             maintainTargetTag(10, 11);
//         }
//         else if (initiallyDetectedTag21 && currentTagID == 20) {
//             maintainTargetTag(21, 20);
//         } */

//         // Gets current robot Pose2d
//         Pose2d currentPose = drivetrain.getState().Pose;
//         Logger.recordOutput("Reefscape/AlignReef/CurrentPose", currentPose);
//         Logger.recordOutput("Reefscape/AlignReef/ExecuteTime", timer.get());
//         // Gets rotational error
//         double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
//         Logger.recordOutput("Reefscape/AlignReef/YawErrfor", yawError);
        
//         // List of X, Y, Yaw velocities to go to target pose
//         double[] velocities;
        
//         // PID Alignment
//         if (usingPID){
//             // Calculates required velocities, rotates before moving
//             velocities = calculateErrorPID(currentPose, true);
//         }
//         // Regular Alignment
//         else{ 
//             // Calculates required velocities, rotates before moving           
//             velocities = calculateError(currentPose, true);
//         }
        
//         // Logs values
//         SmartDashboard.putNumberArray("Target Pose", new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
//         SmartDashboard.putNumberArray("Current Pose", new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
//         SmartDashboard.putNumberArray("Target Vector", new double[]{velocities[0], velocities[1], velocities[2]});
//         Logger.recordOutput("Reefscape/AlignReef/Velocities", velocities);

//         // The direction flips for the red side
//         boolean isFlippingDirection = Constants.contains(new double[]{6, 7, 8, 9, 10, 11}, tID);
//         Logger.recordOutput("Reefscape/AlignReef/FlippedDirection", isFlippingDirection);

//         // Moves the drivetrain
//         if (isFlippingDirection){
//             drivetrain.setControl(driveRequest.withVelocityX(-velocities[0]).withVelocityY(-velocities[1]).withRotationalRate(velocities[2]));
//         }
//         else{
//             drivetrain.setControl(driveRequest.withVelocityX(velocities[0]).withVelocityY(velocities[1]).withRotationalRate(velocities[2]));
//         }

//     }

// /*     // Maintain targeting a specific tag when a different one is detected
//     private void maintainTargetTag(int targetTagID, int ignoredTagID) {
//         System.out.println("Saw tag " + ignoredTagID + ". Gonna continue targeting tag " + targetTagID);
//         Logger.recordOutput("Reefscape/AlignReef/MaintainingTag" + targetTagID, true);
//         tID = targetTagID;
        
//         // Get the target pose for the specified tag
//         double[] tagList = Constants.AprilTagMaps.aprilTagMap.get(targetTagID);
//         if (tagList != null) {
//             // Get the tag pose
//             Pose2d tagPose = new Pose2d(
//                 tagList[0] * Constants.inToM, 
//                 tagList[1] * Constants.inToM, 
//                 new Rotation2d(tagList[3] * Math.PI / 180)
//             );
            
//             // Get the target rotation
//             double targetRotation = tagPose.getRotation().getRadians() - Math.PI;
//             targetRotation = MathUtil.angleModulus(targetRotation);
            
//             // Calculate the rotated offsets
//             double newOffsetX = (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
//             double newOffsetY = (offsetX * Math.sin(targetRotation)) + ((offsetY * Math.cos(targetRotation)));
            
//             // Update the target pose to use the tag's position with the offsets
//             targetPose = new Pose2d(
//                 tagPose.getX() + newOffsetX,
//                 tagPose.getY() + newOffsetY, 
//                 new Rotation2d(targetRotation)
//             );
            
//             // Update PID setpoints
//             pidX.setSetpoint(targetPose.getX());
//             pidY.setSetpoint(targetPose.getY());
//             pidRotate.setSetpoint(targetPose.getRotation().getRadians());
            
//             Logger.recordOutput("Reefscape/AlignReef/UpdatedTargetPose", targetPose);
//         }
//     } */

//     // Calculates the needed velocities to get to the target pose
//     public double[] calculateError(Pose2d currentPose, boolean rotateFirst){
//          // Finds the translation difference (X2-X1, Y2-Y1) between the current and target pose
//          Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
//          // Finds the hypotenuse distance to the desired point
//          double distance = error.getNorm();
//          Logger.recordOutput("Reefscape/AlignReef/TranslationError", new double[]{error.getX(), error.getY()});
//          Logger.recordOutput("Reefscape/AlignReef/Distance", distance);
//          // This gets the robots current rotation (rad)
//          // AngleModulus normalizes the difference to always take the shortest path
//          double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
//          Logger.recordOutput("Reefscape/AlignReef/YawError", yawError);

//          // Intitializes rotation rates
//          double velocityX = 0.0;
//          double velocityY = 0.0;
//          double velocityYaw = 0.0;

//          // Movement Correction
//         if (distance > positionTolerance) {
//             // Normalizes the error vector into a unit vector (value between -1 to 1) and applies the speed
//             // The error vector represent both the direction and magnitude as the same. 
//             velocityX = (error.getX() / distance) * speed;
//             velocityY = (error.getY() / distance) * speed;
//         } 
//         else {
//             // Wont move if within tolerance
//             velocityX = 0;
//             velocityY = 0;
//         }
//         // Rotational Correction 
//         if (Math.abs(yawError) > yawTolerance) {
//             velocityYaw = calculateYawVelocity(yawError);
//             if (rotateFirst){
//                 velocityX = 0;
//                 velocityY = 0;
//             }
//         } 
//         else {
//             // Wont rotate if within tolerance
//             velocityYaw = 0;
//         }
//         // Returns the X, Y, Yaw powers
//         double[] result = new double[]{velocityX, velocityY, velocityYaw};
//         Logger.recordOutput("Reefscape/AlignReef/CalculatedVelocities", result);
//         return result;
//     }   

//     // Calculates the needed velocities to get to the target pose with PID
//     private double[] calculateErrorPID(Pose2d currentPose, boolean rotateFirst){
//         // Calculates the power for X direction and clamp it between -1 and 1
//         double velocityX = pidX.calculate(currentPose.getX());
//         velocityX = MathUtil.clamp(velocityX, -speed, speed);
        
//         // Calculates the power for Y direction and clamp it between -1 and 1
//         double velocityY = pidY.calculate(currentPose.getY());
//         velocityY = MathUtil.clamp(velocityY, -speed, speed);
//         // Calculates the power for the Rotation direction and clamps it between -2 and 2
//         double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
//         velocityYaw = MathUtil.clamp(velocityYaw, -2, 2);
//         // Logs PID values
//         Logger.recordOutput("Reefscape/Limelight/x error", pidX.getError());
//         Logger.recordOutput("Reefscape/Limelight/y error", pidY.getError());
//         Logger.recordOutput("Reefscape/AlignReef/RotationalError", pidRotate.getError());
//         Logger.recordOutput("Reefscape/Limelight/PIDOutputX", velocityX);
//         Logger.recordOutput("Reefscape/Limelight/PIDOutputY", velocityY);
//         Logger.recordOutput("Reefscape/AlignReef/PIDOutputYaw", velocityYaw);

//         // Returns the X, Y, Yaw powers
//         double[] result = new double[]{velocityX, velocityY, velocityYaw};
//         Logger.recordOutput("Reefscape/AlignReef/CalculatedPIDVelocities", result);
//         return result;
//     }

//     // Returns the velocity for the yaw
//     private double calculateYawVelocity(double yawError) {
//         return Math.signum(yawError) * rotationSpeed;
//     }

//     // Called every 20ms to check if command is ended
//     @Override
//     public boolean isFinished(){
//         // If a tag wasn't detected, command will end
//         if (!tagDetected){
//             //return true;
//         }
//         // Without PID, it will check until the tolerance is reached
//         if (!usingPID){
//             Pose2d currentPose = drivetrain.getState().Pose;
//             if (currentPose != null && targetPose != null){
//                 double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
//                 // This gets the yaw error from the target
//                 double yawError = MathUtil.angleModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

//                 // Ends once robot is within tolerance
//                 return distance <= positionTolerance && Math.abs(yawError) <= yawTolerance;
//             }
//             else{
//                 return super.isFinished();
//             }
            
//         }
//         // PID will have its own tolerance check, so isFinished is unnecessary
//         else{
//             return super.isFinished();
//         }
//     }

//     // Called once Command ends
//     @Override
//     public void end(boolean interrupted) {
//         // Ensures drivetrain stop
//         drivetrain.setControl(stop);
//         robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; //remove 0.7
        
// /*         // Reset the tag 10 or tag 21 flag when command ends
//         initiallyDetectedTag10 = false;
//         initiallyDetectedTag21 = false;
//         System.out.println("Reset initiallyDetectedTag10 flag");
//         Logger.recordOutput("Reefscape/AlignReef/InitiallyDetectedTag10", false);
//         System.out.println("Reset initiallyDetectedTag21 flag");
//         Logger.recordOutput("Reefscape/AlignReef/InitiallyDetectedTag21", false); */
        
//         if (interrupted) {
//             System.out.println("AlignReef interrupted.");
//         } else {
//             System.out.println("AlignReef completed.");
//         }
//     }

// }