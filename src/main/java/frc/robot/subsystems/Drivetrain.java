// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainIO.DrivetrainIOInputs;
import frc.robot.subsystems.Transmission.GearState;

public class Drivetrain extends SubsystemBase {
  
  private Supplier<Transmission.GearState> gearStateSupplier;

  // public DifferentialDrive diffDrive;

  // Set up the BuiltInAccelerometer
  public Pigeon2 pigeon = new Pigeon2(Constants.CANBusIDs.kPigeonIMU);

  /**
   * Brings in the swerve module IO class. This gives us direct control to the hardware/simulated swerve module
   */
  private final DrivetrainIO io;
  private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(DrivetrainIO io, Supplier<Transmission.GearState> gearStateSupplier) {
    this.io = io;

    this.gearStateSupplier = gearStateSupplier;

    // Motors
    this.configmotors();

    // Configure PID values for the talons
    this.setWheelPIDF();

    // this.diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Reset encoders and gyro
    this.resetEncoders();
    this.zeroGyro();
  }

  public void setWheelPIDF() {
    // // set the PID values for each individual wheel
    // for (TalonFX fx : new TalonFX[] { this.leftLeader, this.rightLeader }) {
    //   // in init function, set slot 0 gains
    //   var slot0Configs = new Slot0Configs();
    //   slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    //   slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    //   slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    //   slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
    //   slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

    //   fx.getConfigurator().apply(slot0Configs);
    // }
  }

  public void configmotors() { // new
    // // Configure the motors
    // for (TalonFX fx : new TalonFX[] { this.leftLeader, this.leftFollower, this.rightLeader, this.rightFollower }) {    
    //   // Apply default configuration
    //   fx.getConfigurator().apply(new TalonFXConfiguration());     
    // }

    // /* Configure the devices */
    // var leftConfiguration = new TalonFXConfiguration();
    // var rightConfiguration = new TalonFXConfiguration();

    // leftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;  
    // rightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1; 

    // leftConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // rightConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // // Have the wheels on each side of the drivetrain run in opposite directions
    // leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // // Apply the configuration to the wheels
    // this.leftLeader.getConfigurator().apply(leftConfiguration);
    // this.leftFollower.getConfigurator().apply(leftConfiguration);
    // this.rightLeader.getConfigurator().apply(rightConfiguration);
    // this.rightFollower.getConfigurator().apply(rightConfiguration);

    // // Set up followers to follow leaders
    // this.leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    // this.rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
  
    // // Enable safety
    // this.leftLeader.setSafetyEnabled(true);
    // this.rightLeader.setSafetyEnabled(true);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void halt() {
    io.setVoltage(0, 0);
  }

  public void drive(double xSpeed, double zRotation) {

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void zeroGyro() {
    this.pigeon.reset();
  }

  public void resetEncoders() {
    io.resetEncoders();
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
    if (gearState == Transmission.GearState.HIGH) {
      return motorRotations / (DrivetrainConstants.EncoderCPR * DrivetrainConstants.HighGearRatio);
    } else {
      return motorRotations / (DrivetrainConstants.EncoderCPR * DrivetrainConstants.LowGearRatio);
    }
  }

  public double wheelRotationsToMeters(double wheelRotations) {
    return DrivetrainConstants.WheelDiameterMeters * Math.PI * wheelRotations;
  }

  // Encoder ticks to meters
  public double encoderTicksToMeters(double encoderTicks) {
    GearState gearState = this.gearStateSupplier.get();
    return this.wheelRotationsToMeters(this.motorRotationsToWheelRotations(encoderTicks, gearState));
  }

  public double getLeftDistanceMeters() {
    // return this.encoderTicksToMeters(this.leftLeader.getPosition().getValue());
    return this.encoderTicksToMeters(inputs.leftPositionTicks);
  }

  public double getRightDistanceMeters() {
    // return this.encoderTicksToMeters(this.rightLeader.getPosition().getValue());
    return this.encoderTicksToMeters(inputs.rightPositionTicks);
  }

  public double getAvgDistanceMeters() {
    return (this.getLeftDistanceMeters() + this.getRightDistanceMeters()) / 2;
  }

  public double[] readGyro() {
    double[] angle = new double[3];
    angle[0] = this.pigeon.getAccumGyroZ().getValue();
    angle[1] = this.pigeon.getAccumGyroY().getValue();
    angle[2] = this.pigeon.getAccumGyroX().getValue();
    return angle;
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    publishTelemetry();
  }

  public void publishTelemetry() {

  }  
}
