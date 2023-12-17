// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
// import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainIO.DrivetrainIOInputs;
import frc.robot.subsystems.GyroIO.GyroIOInputs;
import frc.robot.subsystems.Transmission.GearState;

public class Drivetrain extends SubsystemBase {
  
  private Supplier<Transmission.GearState> gearStateSupplier;

  // // Set up the BuiltInAccelerometer
  // public Pigeon2 pigeon = new Pigeon2(Constants.CANBusIDs.kPigeonIMU);

  /**
   * Brings in the swerve module IO class. This gives us direct control to the hardware/simulated swerve module
   */
  private final DrivetrainIO io;
  private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();
  private final GyroIO gyroIO; 
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(GyroIO gyroIO,
                    DrivetrainIO drivetrainIO, 
                    Supplier<Transmission.GearState> gearStateSupplier) {
    this.io = drivetrainIO;
    this.gyroIO = gyroIO;

    this.gearStateSupplier = gearStateSupplier;

    // Reset encoders and gyro
    this.resetEncoders();
    this.zeroGyro();
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void halt() {
    io.setVoltage(0, 0);
  }

  public void drive(double xSpeed, double zRotation) {
    io.drive(xSpeed, zRotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void zeroGyro() {
    gyroIO.resetGyro();
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
    return this.encoderTicksToMeters(inputs.leftPositionTicks);
  }

  public double getRightDistanceMeters() {
    return this.encoderTicksToMeters(inputs.rightPositionTicks);
  }

  public double getAvgDistanceMeters() {
    return (this.getLeftDistanceMeters() + this.getRightDistanceMeters()) / 2;
  }

  public double[] readGyro() {
    return gyroInputs.gyroYawPitchRoll;
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive", gyroInputs);

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    publishTelemetry();
  }

  public void publishTelemetry() {
  }  

}
