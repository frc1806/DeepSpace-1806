package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
//import com.revrobotics.CANSparkMax;

public class HABinAGoodTime implements Subsystem {

    private static HABinAGoodTime mHabANiceDay = new HABinAGoodTime();

    private CANSparkMax leftHABArm;
    private CANSparkMax rightHABArm;

    private CANPIDController leftHABArmController;
    private CANPIDController rightHABArmController;


    public static HABinAGoodTime getInstance(){
        return mHabANiceDay;
    }

    private HABinAGoodTime(){
        leftHABArm = new CANSparkMax(RobotMap.HABLiftLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightHABArm = new CANSparkMax(RobotMap.HABLiftRight, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftHABArmController = new CANPIDController(leftHABArm);
        rightHABArmController = new CANPIDController(rightHABArm);
    }

    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Climb rightVel", rightHABArm.getEncoder().getVelocity());
        SmartDashboard.putNumber("Climb leftVel", leftHABArm.getEncoder().getVelocity());
        //SmartDashboard.putNumber("Climb angleOffset", r)
    }

    public void stop(){
        rightHABArm.stopMotor();
        leftHABArm.stopMotor();
    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }


    public void goToHatchMode(){
        //nothing to do here
    }

    public void goToCargoMode(){
        //nothing to do here
    }

    public void retractAll() {
        //TODO
    }
}
