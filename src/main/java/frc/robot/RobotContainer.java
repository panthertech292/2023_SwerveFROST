package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.AutoChooser;
import frc.robot.autos.AutoTrajectories;
import frc.robot.autos.eventMap;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
    private final eventMap map = new eventMap(s_Swerve);
    private final AutoTrajectories trajectories = new AutoTrajectories();
    private final AutoChooser chooser = new AutoChooser(trajectories, map.getMap(), s_Swerve);

    //private String pPlan = null;

    public CommandBase autoCode = Commands.sequence(new PrintCommand("no auto selected"));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driver_AutoBalance = new JoystickButton(driver, XboxController.Button.kB.value);

    //Arm Commands
    //private final Command z_ArmRotateManual = new ArmRotateManual(s_ArmSubsystem, 0);
    /* Variables */
    boolean driveStatus = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      s_ArmSubsystem.setDefaultCommand(
        new ArmRotateManual(s_ArmSubsystem, 
        () -> operator.getRawAxis(XboxController.Axis.kRightY.value)));

      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
        )
      );
        
      // Configure the button bindings
      configureButtonBindings();

      SmartDashboard.putData("Auto Choices", chooser.getAutoChooser());
      
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons (and op buttons) */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }
    
    public void printValues(){
        //SmartDashboard.putNumber("balanceP", 0.03);
        // SmartDashboard.getNumber("balanceI", elevatorAxis);
        // SmartDashboard.getNumber("balanceD", elevatorAxis);

        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());

        SmartDashboard.putNumber("Pid off", chooser.getPIDController().getPositionError());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {

      Constants.gyroOffset = s_Swerve.gyro.getPitch();
      s_Swerve.zeroGyro();
      //s_Swerve.gyro.setYaw(180);
      return chooser.getCommand();
      //return null;
    }
}
