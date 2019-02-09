package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.loop.Looper;
//import com.revrobotics.CANSparkMax;

public class HABinAGoodTime implements Subsystem {

    //CANSparkMax leftHABArm;
    //CANsparkMax rightHABArm;
private static HABinAGoodTime mHabANiceDay = new HABinAGoodTime();



    public static HABinAGoodTime getInstance(){
        return mHabANiceDay;
    }


    public void writeToLog(){

    }

    public void outputToSmartDashboard(){

    }

    public void stop(){

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
