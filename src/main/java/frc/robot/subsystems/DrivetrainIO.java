package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DrivetrainIO { 

  public static class DrivetrainIOInputs implements LoggableInputs {
    public double leftPositionTicks = 0.0; 
    public double rightPositionTicks = 0.0;  
    // public double[] gyroYawPitchRoll;
    // public double gyroYawPosition;

    public void toLog(LogTable table) {
      table.put("leftPositionTicks", leftPositionTicks);   
      table.put("rightPositionTicks", rightPositionTicks);
      // table.put("GyroYawPosition", gyroYawPosition);
    }

    public void fromLog(LogTable table) {
      leftPositionTicks = table.getDouble("leftPositionTicks", leftPositionTicks);    
      rightPositionTicks = table.getDouble("rightPositionTicks", rightPositionTicks);
      // gyroYawPitchRoll = table.getDoubleArray(null, gyroYawPitchRoll);
      // gyroYawPosition = table.getDouble("GyroYawPosition", gyroYawPosition);
    }
  }  

  // Updates the set of loggable inputs. 
  public default void updateInputs(DrivetrainIOInputs inputs) {}

  // Create hardware Control Input functions
  public default void drive(double xSpeed, double zRotation) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void resetEncoders() {}

  // public default void resetGyro() {}

}
