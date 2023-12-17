package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SimulationConstants;

public class DrivetrainIOSim implements DrivetrainIO{

  private final TalonFX leftLeader = new TalonFX(1);
  private final TalonFX rightLeader = new TalonFX(2);
  private final TalonFXSimState leftSim = leftLeader.getSimState();
  private final TalonFXSimState rightSim = rightLeader.getSimState();

  private final DifferentialDrivetrainSim driveSim = SimulationConstants.driveSim;
  
  //  /* Simulation model of the drivetrain */
  // private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
  //     DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
  //     DrivetrainConstants.LowGearRatio, 
  //     2.1, // MOI of 2.1 kg m^2 (from CAD model).
  //     26.5, // Mass of the robot is 26.5 kg.
  //     DrivetrainConstants.WheelDiameterMeters/2, // Robot uses 3" radius (6" diameter) wheels.
  //     DrivetrainConstants.kTrackWidthMeters, // Distance between wheels is _ meters.

  //     /*
  //     * The standard deviations for measurement noise:
  //     * x and y: 0.001 m
  //     * heading: 0.001 rad
  //     * l and r velocity: 0.1 m/s
  //     * l and r position: 0.005 m
  //     */
  //     /* Uncomment the following line to add measurement noise. */
  //     null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  // );

  /**
	 * Creates a new drivebase simualtor using Falcon 500 motors.
	 */
	public DrivetrainIOSim() {
		StatusCode returnCode;

    TalonFXConfiguration fxCfg = new TalonFXConfiguration();
    fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    do {
        returnCode = leftLeader.getConfigurator().apply(fxCfg);
    } while(!returnCode.isOK());

    fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    do {
        returnCode = rightLeader.getConfigurator().apply(fxCfg);
    } while(!returnCode.isOK());

    leftLeader.getPosition().setUpdateFrequency(100);
    rightLeader.getPosition().setUpdateFrequency(100);
	}

  public void updateInputs(DrivetrainIOInputs inputs) {
    /* Pass the robot battery voltage to the simulated devices */
    this.leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.driveSim.setInputs(this.leftSim.getMotorVoltage(),
                            this.rightSim.getMotorVoltage());

    /*
      * Advance the model by 20 ms. Note that if you are running this
      * subsystem in a separate thread or have changed the nominal
      * timestep of TimedRobot, this value needs to match it.
    */
    this.driveSim.update(0.02);

    // Read the encoder positions
    inputs.leftPositionTicks = this.leftLeader.getPosition().getValue();
    inputs.rightPositionTicks = this.rightLeader.getPosition().getValue();
  }
  
}
