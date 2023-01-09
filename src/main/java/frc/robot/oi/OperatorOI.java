// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OperatorOI {
    private XboxController m_controller;

    public OperatorOI(XboxController controller) {
        m_controller = controller;
    }

    // ---------------- Drivetrain ----------------------------

    public Trigger getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }

    public Trigger getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }

}
