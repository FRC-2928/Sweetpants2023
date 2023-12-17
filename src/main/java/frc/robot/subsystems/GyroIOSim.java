package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.SimulationConstants;

public class GyroIOSim implements GyroIO {
  private final Pigeon2 imu = new Pigeon2(0);
  private final Pigeon2SimState imuSim = imu.getSimState();

  private final DifferentialDrivetrainSim driveSim = SimulationConstants.driveSim;

  public GyroIOSim() {
    Pigeon2Configuration imuCfg = new Pigeon2Configuration();
    imu.getConfigurator().apply(imuCfg);
    imu.getYaw().setUpdateFrequency(100);
  }

  public void updateInputs(GyroIOInputs inputs) {
    imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // driveSim.setInputs(leftSim.getMotorVoltage(),
    //             rightSim.getMotorVoltage());

    /*
      * Advance the model by 20 ms. Note that if you are running this
      * subsystem in a separate thread or have changed the nominal
      * timestep of TimedRobot, this value needs to match it.
    */
    this.driveSim.update(0.02);

    imuSim.setRawYaw(driveSim.getHeading().getDegrees());

    inputs.gyroYawPosition = driveSim.getHeading().getDegrees();

  }

}
