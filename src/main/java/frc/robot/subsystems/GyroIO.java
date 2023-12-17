package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;

public interface GyroIO {

  public static class GyroIOInputs implements LoggableInputs {

    public double[] gyroYawPitchRoll;
    public double gyroYawPosition;

    // Updates a LogTable with the data to log.
    public void toLog(LogTable table) {
      table.put("GyroYawPosition", gyroYawPosition);
    }

    // Updates data based on a LogTable.
    public void fromLog(LogTable table) {
      gyroYawPitchRoll = table.getDoubleArray(null, gyroYawPitchRoll);
      gyroYawPosition = table.getDouble("GyroYawPosition", gyroYawPosition);
    }

  }  

  // Updates the set of loggable inputs. 
  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetGyro() {}
  
}
