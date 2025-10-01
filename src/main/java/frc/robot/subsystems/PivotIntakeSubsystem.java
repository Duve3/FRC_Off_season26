package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    private final CANrange coralSensor = new CANrange(PivotIntakeConstants.CORAL_SENSOR_ID);
    
    // PID controller for pivot positioning
    private final PIDController pivotPID = new PIDController(
        PivotIntakeConstants.PIVOT_KP, 
        PivotIntakeConstants.PIVOT_KI, 
        PivotIntakeConstants.PIVOT_KD
    );
    
    // Current pivot setpoint
    private double currentSetpoint = PivotIntakeConstants.STOWED_POSITION;
    
    public PivotIntakeSubsystem() {
        configurePivotMotor();
        configureIntakeMotor();
        
        // Set initial setpoint
        pivotPID.setSetpoint(PivotIntakeConstants.STOWED_POSITION);
        pivotPID.setTolerance(PivotIntakeConstants.PIVOT_TOLERANCE);
        
        // Reset pivot encoder if needed
        pivotEncoder.setPosition(0);
    }
    
    private void configurePivotMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limits for pivot motor
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Limits to prevent over-rotation
        //config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.56; // Slightly past intake position
        //config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.02; // Slightly past stowed
        //config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        pivotMotor.getConfigurator().apply(config);
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
        //currentSetpoint = MathUtil.clamp(setpoint, -0.44, 0);
        currentSetpoint = setpoint;
        pivotPID.setSetpoint(currentSetpoint);
    }
    
    // Get current pivot position
    public double getPivotPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }
    
    // Check if pivot is at setpoint
    public boolean isPivotAtSetpoint() {
        return pivotPID.atSetpoint();
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
        // Update pivot motor using PID
        double pivotPower = pivotPID.calculate(getPivotPosition());
        pivotMotor.set(MathUtil.clamp(pivotPower, -0.44, 0.44));
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
        SmartDashboard.putNumber("Pivot Setpoint", currentSetpoint);
        SmartDashboard.putBoolean("Pivot At Setpoint", isPivotAtSetpoint());
        SmartDashboard.putBoolean("Coral Detected", isCoralDetected());
        //SmartDashboard.putNumber("Coral Distance (mm)", getCoralDistance());
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelMotor.get());
        SmartDashboard.putNumber("Pivot Motor Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Current", intakeWheelMotor.getSupplyCurrent().getValueAsDouble());
    }
}
