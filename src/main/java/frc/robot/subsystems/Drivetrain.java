// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import frc.robot.subsystems.Transmission.GearState;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.FollowerType;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.PigeonIMU;

public class Drivetrain extends SubsystemBase {
  public final TalonFX leftLeader = new TalonFX(Constants.CANBusIDs.DrivetrainLeftBackTalonFX);
  public final TalonFX rightLeader = new TalonFX(Constants.CANBusIDs.DrivetrainRightBackTalonFX);
  public final TalonFX leftFollower = new TalonFX(Constants.CANBusIDs.DrivetrainLeftFrontTalonFX);
  public final TalonFX rightFollower = new TalonFX(Constants.CANBusIDs.DrivetrainRightFrontTalonFX);

  private Supplier<Transmission.GearState> gearStateSupplier;

  public DifferentialDrive diffDrive;

  // Set up the BuiltInAccelerometer
  public Pigeon2 pigeon = new Pigeon2(Constants.CANBusIDs.kPigeonIMU);

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {
    this.gearStateSupplier = gearStateSupplier;

    // Motors
    this.configmotors();

    // Configure PID values for the talons
    // this.setWheelPIDF();

    this.diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Reset encoders and gyro
    this.resetEncoders();
    this.zeroGyro();
  }

  // public void setWheelPIDF() {
  //   // set the PID values for each individual wheel
  //   for (TalonFX fx : new TalonFX[] { this.leftLeader, this.rightLeader }) {
  //     fx.config_kP(0, DrivetrainConstants.GainsProfiled.P, 0);
  //     fx.config_kI(0, DrivetrainConstants.GainsProfiled.I, 0);
  //     fx.config_kD(0, DrivetrainConstants.GainsProfiled.D, 0);
  //     fx.config_kF(0, DrivetrainConstants.GainsProfiled.F, 0);
  //     // m_talonsMaster.config_IntegralZone(0, 30);
  //   }
  // }

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

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    this.leftLeader.getConfigurator().apply(leftConfiguration);
    this.leftFollower.getConfigurator().apply(leftConfiguration);
    this.rightLeader.getConfigurator().apply(rightConfiguration);
    this.rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    this.leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    this.rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
  
    this.leftLeader.setSafetyEnabled(true);
    this.rightLeader.setSafetyEnabled(true);
    

    // Setting followers, followers don't automatically follow the Leader's inverts
    // so you must set the invert type to Follow the Leader
    // this.leftFollower.setInverted(InvertType.FollowMaster);
    // this.rightFollower.setInverted(InvertType.FollowMaster);

    // this.leftFollower.follow(this.leftLeader, FollowerType.PercentOutput);
    // this.rightFollower.follow(this.rightLeader, FollowerType.PercentOutput);

    // this.rightLeader.setInverted(InvertType.InvertMotorOutput);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void halt() {
    this.diffDrive.arcadeDrive(0, 0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var leftSteerRequest = new DutyCycleOut(leftVolts);
    // leftSteerRequest.Output = leftVolts;
    this.leftLeader.setControl(leftSteerRequest);

    var rightSteerRequest = new DutyCycleOut(rightVolts);
    this.rightLeader.setControl(rightSteerRequest);

    // this.leftLeader.set(ControlMode.PercentOutput, leftVolts / 12);
    // this.rightLeader.set(ControlMode.PercentOutput, rightVolts / 12);
    this.diffDrive.feed();
  }

  public void zeroGyro() {
    this.pigeon.reset();
  }

  public void resetEncoders() {
    this.leftLeader.setRotorPosition(0);
    this.rightLeader.setRotorPosition(0);
    // this.leftLeader.setSelectedSensorPosition(0);
    // this.rightLeader.setSelectedSensorPosition(0);
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
    return this.encoderTicksToMeters(this.leftLeader.getPosition().getValue());
  }

  public double getRightDistanceMeters() {
    return this.encoderTicksToMeters(this.rightLeader.getPosition().getValue());
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
    publishTelemetry();
  }

  public void publishTelemetry() {

  }  
}
