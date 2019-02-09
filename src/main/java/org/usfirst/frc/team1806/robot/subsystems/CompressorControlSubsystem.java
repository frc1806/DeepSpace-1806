package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.driver.AnalogPressureSensor;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.Compressor;
import org.usfirst.frc.team1806.robot.util.SamplingFilter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CompressorControlSubsystem implements Subsystem {

    private static CompressorControlSubsystem compressorControlSubsystem = new CompressorControlSubsystem();

    public static CompressorControlSubsystem getInstance(){
        return compressorControlSubsystem;
    }

    Compressor compressor;

    private double currentTimeStamp;
    private double lastTimeStamp;
    private AnalogPressureSensor pressureSensor;
    private Boolean override;
    private PowerDistributionPanel powerDistributionPanel;
    private Double batteryCoulombCount;

    private Boolean wasCompressorRunning;


    private Loop mLoop = new Loop() {

        @Override
        public synchronized void onStart(double timestamp) {
            lastTimeStamp = currentTimeStamp;
            currentTimeStamp = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            // TODO Auto-generated method stub
            lastTimeStamp = currentTimeStamp;
            currentTimeStamp = timestamp;
            synchronized (CompressorControlSubsystem.this) {

                //IF statements in reverse order of priority

                Boolean runCompressor = false;
                if(pressureSamplingFilter.update(pressureSensor.getPressure()) < Constants.kPressureAverageMinimumToStart){
                    runCompressor = true;
                }

                if(voltageSamplingFilter.update(powerDistributionPanel.getVoltage()) < Constants.kBatteryVoltageCompressorShutoffThreshold){
                    runCompressor = false;
                }

                if(amperageSamplingFilter.update(powerDistributionPanel.getTotalCurrent()) > Constants.kAverageAmpDemandToShutOffCompressor) {
                    runCompressor = false;
                }

                if(override){
                    runCompressor = true;
                }

                if((wasCompressorRunning && powerDistributionPanel.getTotalCurrent() > Constants.kAbsoluteRobotCompressorShutOffAmps) ||
                        (!wasCompressorRunning && powerDistributionPanel.getTotalCurrent() > Constants.kAbsoluteRobotCompressorShutOffAmps - Constants.kExpectedCompressorMaxCurrentDraw)){
                    runCompressor = false;
                }

                if((wasCompressorRunning && powerDistributionPanel.getVoltage() < Constants.kBatteryVoltageAbsoluteCompressorShutoffThreshold) ||
                        (!wasCompressorRunning && powerDistributionPanel.getVoltage() < Constants.kBatteryVoltageAbsoluteCompressorShutoffThreshold + Constants.kExpectedCompressorVoltageDrop)) {
                    runCompressor = false;
                }

                compressor.setClosedLoopControl(runCompressor);

                if(powerDistributionPanel.getTotalCurrent()<Constants.kLowAmpLoad){
                    batteryCoulombCount = ((powerDistributionPanel.getVoltage() - Constants.kBatteryDepletedVoltage)/(Constants.kBatteryFullChargeVoltage -Constants.kBatteryDepletedVoltage))* Constants.kFullChargeBatteryCoulombCount;
                }
                else {
                    batteryCoulombCount -= powerDistributionPanel.getTotalCurrent() * (currentTimeStamp - lastTimeStamp);
                }

                }
            }

            @Override
        public synchronized void onStop(double timestamp){
                lastTimeStamp = currentTimeStamp;
                currentTimeStamp = timestamp;
            }


        };

    SamplingFilter pressureSamplingFilter;
    SamplingFilter voltageSamplingFilter;
    SamplingFilter amperageSamplingFilter;



    private CompressorControlSubsystem(){
        compressor = new Compressor(0);
        pressureSensor = new AnalogPressureSensor(0);//TODO put in robot map
        pressureSamplingFilter = new SamplingFilter(Constants.kPressureSensorSamplingLoops);
        voltageSamplingFilter = new SamplingFilter(Constants.kBatteryVoltageSamplingLoops);
        amperageSamplingFilter = new SamplingFilter(Constants.kRobotDemandAmpsSamplingLoops);

    }
    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("STORAGE PRESSURE", pressureSensor.getPressure());
        SmartDashboard.putNumber("Recent Average Pressure", pressureSamplingFilter.getCurrentAverage());
        SmartDashboard.putNumber("Compressor Current", compressor.getCompressorCurrent());
        SmartDashboard.putNumber("Recent Average Battery Voltage", voltageSamplingFilter.getCurrentAverage());
        SmartDashboard.putNumber("Recent Average PDP Total Current", amperageSamplingFilter.getCurrentAverage());
        SmartDashboard.putBoolean("Is Compressor Running?", compressor.getClosedLoopControl());
        SmartDashboard.putNumber("Battery Charge", (batteryCoulombCount/Constants.kFullChargeBatteryCoulombCount)* 100);
    }

    public void stop(){
        compressor.setClosedLoopControl(false);
    }

    public void zeroSensors(){
        //Can't zero an analog sensor, sorry.

    }

    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(mLoop);
    }

    public void setOverride(boolean override){
        this.override = override;
    }



    public void goToHatchMode(){
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }

    public void goToCargoMode(){
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }

    public void retractAll() {
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }
}
