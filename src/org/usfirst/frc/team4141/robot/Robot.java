/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4141.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.  
 *
 * Tweak the PID gains accordingly.
 */

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class Robot extends IterativeRobot {

	TalonSRX talon = new TalonSRX(1);
	Joystick joystick = new Joystick(0);
	StringBuilder stringBuffer = new StringBuilder();
	int nLoops = 0;
	boolean PrevPressWasButton1 = false;
	/** save the target position to servo to */
	double targetPositionRotations;

	public void robotInit() {

		/* choose the sensor and sensor direction */
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		talon.setSensorPhase(Constants.kSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		talon.setInverted(Constants.kMotorInvert);

		/* set the peak and nominal outputs, 12V means full */
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		talon.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();
		/* mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase)
			absolutePosition *= -1;
		if (Constants.kMotorInvert)
			absolutePosition *= -1;
		/* set the quadrature (relative) sensor to match absolute */
		talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	public void disabledPeriodic() {
		commonLoop();
	}
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		commonLoop();
	}
	void commonLoop() {
		/* get gamepad axis */
		double leftYstick = joystick.getY();
		double motorOutput = talon.getMotorOutputPercent();
		boolean button1 = joystick.getRawButton(1);
		boolean button2 = joystick.getRawButton(2);
		/* deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* within 10% of zero */
			leftYstick = 0;

		}
		
		SmartDashboard.putNumber("leftYstick", leftYstick);
		SmartDashboard.putNumber("motorOutput", motorOutput);
		SmartDashboard.putBoolean("button1", button1);
		SmartDashboard.putBoolean("button2", button2);
		SmartDashboard.putBoolean("Previous Press was Button1", PrevPressWasButton1);
		
		
		/* prepare line to print */
		stringBuffer.append("\tout:");
		/* cast to int to remove decimal places */
		stringBuffer.append((int) (motorOutput * 100));
		stringBuffer.append("%"); /* perc */

		stringBuffer.append("\tpos:");
		stringBuffer.append(talon.getSelectedSensorPosition(0));
		stringBuffer.append("u"); /* units */

		/* on button1 press enter closed-loop mode on target position */
		if (!PrevPressWasButton1 && button1) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = leftYstick * 10.0 * 4096;
			talon.set(ControlMode.Position, targetPositionRotations);
			
			SmartDashboard.putNumber("targetPositionRotations", targetPositionRotations);

			stringBuffer.append("Button 1 pressed\n");

		}
		/* on button2 just straight drive */
		if (button2) {
			/* Percent voltage mode */
			talon.set(ControlMode.PercentOutput, leftYstick);
			stringBuffer.append("Button 2 pressed\n");			
		}
		
		/* if Talon is in position closed-loop, print some more info */
		if (talon.getControlMode() == ControlMode.Position) {
			/* append more signals to print when in speed mode. */
			stringBuffer.append("\terr:");
			
			int currentClosedLoopError = talon.getClosedLoopError();

			
			stringBuffer.append(currentClosedLoopError);
			stringBuffer.append("u"); /* units */

			stringBuffer.append("\ttrg:");
			stringBuffer.append(targetPositionRotations);
			stringBuffer.append("u"); /* units */
			
			SmartDashboard.putNumber("currentLoopError", currentClosedLoopError);
		}
		/*
		 * print every ten loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++nLoops >= 10) {
			nLoops = 0;
			System.out.println(stringBuffer.toString());
		}
		stringBuffer.setLength(0);
		/* save button state for on press detect */
		PrevPressWasButton1 = button1;
	}
}