package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;

public class GyroIOPideon2 implements GyroIO {

  // Set up the Gyro
  public Pigeon2 pigeon = new Pigeon2(Constants.CANBusIDs.kPigeonIMU);
  
  public GyroIOPideon2() {

  }

  public double[] readGyro() {
    double[] angle = new double[3];
    angle[0] = this.pigeon.getAccumGyroZ().getValue();
    angle[1] = this.pigeon.getAccumGyroY().getValue();
    angle[2] = this.pigeon.getAccumGyroX().getValue();
    return angle;
  }

  // Updates the set of loggable inputs. 
  public void updateInputs(GyroIOInputs inputs) {
     inputs.gyroYawPosition = this.pigeon.getAccumGyroZ().getValue();
     inputs.gyroYawPitchRoll = readGyro();
  }
 

  @Override
  public void resetGyro() {
    this.pigeon.reset();
  }

}
