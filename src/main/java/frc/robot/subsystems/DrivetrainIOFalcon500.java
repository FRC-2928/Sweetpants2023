package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DrivetrainIOFalcon500 implements DrivetrainIO {
  public final TalonFX leftLeader = new TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
  public final TalonFX rightLeader = new TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
  public final TalonFX leftFollower = new TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
  public final TalonFX rightFollower = new TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);
  private DifferentialDrive diffDrive;


  // private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  public DrivetrainIOFalcon500() {
    configmotors();
    setWheelPIDF();
    this.diffDrive = new DifferentialDrive(leftLeader, rightLeader);
  } 

  public void configmotors() { // new
    // Configure the motors
    for (TalonFX fx : new TalonFX[] { this.leftLeader, this.leftFollower, this.rightLeader, this.rightFollower }) {    
      // Apply default configuration
      fx.getConfigurator().apply(new TalonFXConfiguration());     
    }

    /* Configure the devices */
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    leftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;  
    rightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1; 

    leftConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rightConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Have the wheels on each side of the drivetrain run in opposite directions
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply the configuration to the wheels
    this.leftLeader.getConfigurator().apply(leftConfiguration);
    this.leftFollower.getConfigurator().apply(leftConfiguration);
    this.rightLeader.getConfigurator().apply(rightConfiguration);
    this.rightFollower.getConfigurator().apply(rightConfiguration);

    // Set up followers to follow leaders
    this.leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    this.rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
  
    // Enable safety
    this.leftLeader.setSafetyEnabled(true);
    this.rightLeader.setSafetyEnabled(true);
  }

  public void setWheelPIDF() {
    // set the PID values for each individual wheel
    for (TalonFX fx : new TalonFX[] { this.leftLeader, this.rightLeader }) {
      // in init function, set slot 0 gains
      var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
      slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

      fx.getConfigurator().apply(slot0Configs);
    }
  }

  public void updateInputs(DrivetrainIOInputs inputs) {
    inputs.leftPositionTicks = this.leftLeader.getPosition().getValue();
    inputs.rightPositionTicks = this.rightLeader.getPosition().getValue();
  }

  // Implement hardware Control Input functions
  @Override
  public void drive(double xSpeed, double zRotation) {
    diffDrive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    var leftVoltsRequest = new DutyCycleOut(leftVolts / 12);
    // leftVoltsRequest.Output = leftVolts / 12;
    this.leftLeader.setControl(leftVoltsRequest);

    var rightVoltsRequest = new DutyCycleOut(rightVolts / 12);
    this.rightLeader.setControl(rightVoltsRequest);

    this.diffDrive.feed();
  }

  @Override
  public void resetEncoders() {
    this.leftLeader.setRotorPosition(0);
    this.rightLeader.setRotorPosition(0);
  }

}
