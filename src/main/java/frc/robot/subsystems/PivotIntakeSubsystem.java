package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotIntakeConstants;

public class PivotIntakeSubsystem extends SubsystemBase {

    // Motors and sensors
    private final TalonFX pivotMotor = new TalonFX(PivotIntakeConstants.PIVOT_MOTOR_ID);
    private final TalonFX intakeWheelMotor = new TalonFX(PivotIntakeConstants.INTAKE_WHEEL_MOTOR_ID);
    private final CANcoder pivotEncoder = new CANcoder(PivotIntakeConstants.PIVOT_ENCODER_ID);
    //private final CANrange coralSensor = new CANrange(PivotIntakeConstants.CORAL_SENSOR_ID);
    
    // Position control request for pivot
    private final PositionVoltage pivotPositionControl = new PositionVoltage(0).withSlot(0);
    
    // Current pivot setpoint
    private double currentSetpoint = PivotIntakeConstants.STOWED_POSITION;
    
    public PivotIntakeSubsystem() {
        configurePivotMotor();
        configureIntakeMotor();
        
        // Reset pivot encoder to 0 at startup
        pivotEncoder.setPosition(0);
        
        // Sync the TalonFX position with the CANcoder
        // This ensures the motor controller knows the current position
        pivotMotor.setPosition(0);
    }
    
    private void configurePivotMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // PID Configuration for position control
        config.Slot0.kP = PivotIntakeConstants.PIVOT_KP;
        config.Slot0.kI = PivotIntakeConstants.PIVOT_KI;
        config.Slot0.kD = PivotIntakeConstants.PIVOT_KD;
        config.Slot0.kV = PivotIntakeConstants.PIVOT_KV;  // Feedforward velocity
        config.Slot0.kS = PivotIntakeConstants.PIVOT_KS;  // Feedforward static
        config.Slot0.kG = PivotIntakeConstants.PIVOT_KG;  // Gravity compensation
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine; // For pivoting arms
        
        // Use the remote CANcoder for absolute position feedback
        config.Feedback.FeedbackRemoteSensorID = PivotIntakeConstants.PIVOT_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0; // Adjust if there's gearing
        config.Feedback.RotorToSensorRatio = 1.0; // Not really sure if needed but included anyways
        
        // Current limits for pivot motor
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Limits to prevent over-rotation
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.56; // Slightly past intake position
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.02; // Slightly past stowed
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        pivotMotor.getConfigurator().apply(config);
        
        // Set position update frequency for better feedback
        pivotMotor.getPosition().setUpdateFrequency(100);
        pivotEncoder.getPosition().setUpdateFrequency(100);
        pivotMotor.optimizeBusUtilization();
    }
    
    private void configureIntakeMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limits for intake motor
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        intakeWheelMotor.getConfigurator().apply(config);
    }
    
    // Set the pivot position setpoint
    public void setPivotSetpoint(double setpoint) {
        currentSetpoint = setpoint;
        // Command the motor to the target position
        pivotMotor.setControl(pivotPositionControl.withPosition(currentSetpoint));
    }
    
    // Get current pivot position
    public double getPivotPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }
    
    // Check if pivot is at setpoint
    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotPosition() - currentSetpoint) < PivotIntakeConstants.PIVOT_TOLERANCE;
    }
    
    // Run intake wheels
    public void setIntakeSpeed(double speed) {
        intakeWheelMotor.set(speed);
    }
    
    // Check if coral is detected by CanRange sensor
    public boolean isCoralDetected() {
        //return coralSensor.getDistance().getValueAsDouble() < PivotIntakeConstants.CORAL_DETECTED_DISTANCE_MM;
        return false;
    }
    
    // Get the distance reading from CanRange sensor
    //public double getCoralDistance() {
        //return coralSensor.getDistance().getValueAsDouble();
        //return 0d;
    //}
    
    // COMMAND METHODS
    
    // Command to move pivot to stowed position
    public Command stowPivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION));
    }
    
    // Command to move pivot to intake position
    public Command deployPivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION));
    }
    
    // Command to move pivot to reef scoring position
    public Command intermediatePivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.REEF_SCORING_POSITION));
    }
    
    // Command to run intake wheels forward
    public Command intakeWheels() {
        return Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED), this);
    }
    
    // Command to run intake wheels in reverse (for scoring)
    public Command reverseIntakeWheels() {
        return Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this);
    }
    
    // Command to stop intake wheels
    public Command stopWheels() {
        return Commands.run(() -> setIntakeSpeed(0), this);
    }
    
    @Override
    public void periodic() {
        // Motor is controlled via position control set in setPivotSetpoint()
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
        SmartDashboard.putNumber("Pivot Setpoint", currentSetpoint);
        SmartDashboard.putNumber("Pivot Error", currentSetpoint - getPivotPosition());
        SmartDashboard.putBoolean("Pivot At Setpoint", isPivotAtSetpoint());
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        //SmartDashboard.putNumber("Coral Distance (mm)", getCoralDistance());
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelMotor.get());
        SmartDashboard.putNumber("Pivot Motor Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Position (internal)", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Duty Cycle", pivotMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Current", intakeWheelMotor.getSupplyCurrent().getValueAsDouble());
    }
}
