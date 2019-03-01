package org.usfirst.frc.team1806.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.auto.*;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.path.motion.RobotStateEstimator;
import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.util.CrashTracker;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    public static OI m_oi;
    public static PowerDistributionPanel powerDistributionPanel;
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    private static final SubsystemManager S_SubsystemManager = new SubsystemManager(
            Arrays.asList(DriveTrainSubsystem.getInstance(), LiftSubsystem.getInstance(), CompressorControlSubsystem.getInstance(), CargoIntakeSubsystem.getInstance(), SonarPositioningSubsystem.getInstance()));

    private Looper mEnabledLooper = new Looper();

    public static AutoModeBase selectedAuto;
    public static boolean isBlue;
    public boolean arePathsInit = false;
    public UsbCamera camera;
    public MjpegServer cameraServer;
    public static boolean needToPositionControlInTele = false;
    public enum AutoInTeleOp{
      AUTO_DISABLED,
      AUTO_INIT,
      AUTO_PERIODIC
    }
    public AutoInTeleOp autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;

    public enum GamePieceMode{
        HATCH_PANEL,
        CARGO
    }

    private static GamePieceMode GamePieceMode;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Robot.setGamePieceMode(GamePieceMode.HATCH_PANEL);
      m_oi = new OI();

      //Sets up camera
      camera = new UsbCamera("cam0", 0);
      camera.setFPS(30);
      camera.setBrightness(20);
      camera.getPath();
      cameraServer = new MjpegServer("camera",  5806);
      cameraServer.setSource(camera);
      zeroAllSensors();

      //adds in the iterative code to make the code run
      mEnabledLooper.register(RobotStateEstimator.getInstance());
      S_SubsystemManager.registerEnabledLoops(mEnabledLooper);

      powerDistributionPanel = new PowerDistributionPanel();
      SmartDashboard.putData("Auto mode", m_chooser);
      mAutoModeExecuter = null;
      mAutoModeExecuter = new AutoModeExecuter();
      //mAutoModeExecuter.setAutoMode(new QualMode()); TODO
      mDrive.setCoastMode();
      AutoModeSelector.initAutoModeSelector();
      needToPositionControlInTele = false;
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e){
        System.out.println(e);
      }

      BluePathAdapter.initPaths();
      RedPathAdapter.initPaths();
      SmartDashboard.putString("testingFieldValue", "LLR");
    }


    @Override
    public void disabledInit() {
      mEnabledLooper.stop();
      if(mAutoModeExecuter != null) {
        mAutoModeExecuter.stop();
      }
      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
      m_oi.resetAutoLatch();
    }

    @Override
    public void disabledPeriodic() {
      if(DriverStation.getInstance().isDSAttached() ){
        if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
          isBlue = true;
        } else if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red){
          isBlue = false;
        }
      }
      allPeriodic();
      AutoModeSelector.initAutoModeSelector();
      selectedAuto = AutoModeSelector.getSelectedAutoMode();
      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;

    }


    @Override
    public void autonomousInit() {
      try {
			zeroAllSensors();
			CrashTracker.logAutoInit();
            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            //mDrive.setHighGear(true);
        needToPositionControlInTele = false;
        mDrive.setBrakeMode();
        mEnabledLooper.start();
        mAutoModeExecuter.setAutoMode(selectedAuto);
        mAutoModeExecuter.start();
      } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
      }

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
      allPeriodic();
      Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
      mEnabledLooper.start();
      if(mAutoModeExecuter != null) {
        mAutoModeExecuter.stop();
        mAutoModeExecuter = null;
        mAutoModeExecuter = new AutoModeExecuter();
      }
      mDrive.setOpenLoop(DriveSignal.NEUTRAL);
      mDrive.setNeutralMode(false);
      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
    }


    @Override
    public void teleopPeriodic() {
      if(Constants.enableAutoInTeleOp){
        switch(autoInteleOpState){
          case AUTO_DISABLED:
            if(m_oi.autoInTeleOpOn()){
              autoInteleOpState = AutoInTeleOp.AUTO_INIT;
            } else {
              runTeleOp();
            }
            break;
          case AUTO_INIT:
            selectedAuto = AutoModeSelector.getSelectedAutoMode();
            if(m_oi.autoInTeleOpOn()){
              zeroAllSensors();
              if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();

              }
              autonomousInit();
              m_oi.autoRunCommands();
              autoInteleOpState = AutoInTeleOp.AUTO_PERIODIC;
            } else {
              autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
              teleopInit();
            }
            break;
          case AUTO_PERIODIC:
            if(m_oi.autoInTeleOpOn()){
              autonomousPeriodic();
              m_oi.autoRunCommands();
            } else {
              autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
              teleopInit();
            }
            break;
          default:
            runTeleOp();
            break;
        }

      } else {
        runTeleOp();
      }
    }

    @Override
    public void testPeriodic() {
    }


    public void zeroAllSensors() {
//		System.out.println("Zeroing all Sensors..");
      S_SubsystemManager.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
//        System.out.print("All Sensors zeroed!");

}
    public synchronized void allPeriodic() {
      S_SubsystemManager.outputToSmartDashboard();
      mRobotState.outputToSmartDashboard();
      mEnabledLooper.outputToSmartDashboard();
      SmartDashboard.putString("Auto We Are Running", AutoModeSelector.returnNameOfSelectedAuto());
      SmartDashboard.putNumber("PDP Total", powerDistributionPanel.getTotalCurrent());
    }
    private void runTeleOp(){
      Scheduler.getInstance().run();
      m_oi.runCommands();
      allPeriodic();
    }

    /**
     * Sets the global game piece mode, runs functions associated with mode change on subsystems.
     * @param mode the wanted mode
     */
    public static synchronized void setGamePieceMode(GamePieceMode mode){
        GamePieceMode = mode;

        switch(mode){
            case CARGO:
                S_SubsystemManager.goToCargoMode();
                break;
            case HATCH_PANEL:
                S_SubsystemManager.goToCargoMode();
                break;
            default:
                break;
        }
    }

    public static synchronized GamePieceMode getGamePieceMode(){
        return GamePieceMode;
    }

    public static synchronized void RetractAll(){
        S_SubsystemManager.retractAll();
    }
  }
