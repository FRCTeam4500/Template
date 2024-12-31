package frc.robot.hardware;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** A class representing a motor */
public class Motor extends SubsystemBase implements Loggable {
  private double target;
  private boolean useVoltage;
  private TargetType type;
  private DoubleConsumer positionSetter;
  private DoubleConsumer voltageSetter;
  private DoubleSupplier positionGetter;
  private DoubleSupplier velocityGetter;
  private FeedbackController fb;
  private FeedforwardConstants ff;
  private Loggable motorInfo;

  /**
   * Creates a new motor where the given parameters are used to interface with the hardware or sim
   *
   * <p><strong> You probably want to use one of the factory methods, rather than the constructor
   * directly!!</strong>
   *
   * @param type The type of target the motor is trying to reach
   * @param positionSetter A function that sets the current position of the motor to the passed in
   *     value
   * @param voltageSetter A function that applies the passed in value as voltage to the motor
   * @param positionGetter A function that returns the current position of the motor
   * @param velocityGetter A function that returns the current velocity of the motor, in units per
   *     second
   * @param fb A feedback controller, which drives the motor to its goal
   * @param ff An optional set of feedforward constants, used to help the feedback controller
   * @param motorInfo A {@link Loggable} which logs information about the motor, such as applied
   *     voltage, temperature, and current
   */
  public Motor(
      TargetType type,
      DoubleConsumer positionSetter,
      DoubleConsumer voltageSetter,
      DoubleSupplier positionGetter,
      DoubleSupplier velocityGetter,
      FeedbackController fb,
      Optional<FeedforwardConstants> ff,
      Loggable motorInfo) {
    target = 0;
    useVoltage = true;
    this.type = type;
    this.positionSetter = positionSetter;
    this.voltageSetter = voltageSetter;
    this.positionGetter = positionGetter;
    this.velocityGetter = velocityGetter;
    this.fb = fb;
    this.ff = ff.orElse(new FeedforwardConstants(0, 0, 0, 0));
    this.motorInfo = motorInfo;
  }

  /**
   * Sets the target position/velocity for the motor.
   *
   * @param nextTarget the target position/velocity to go to
   */
  public void setTarget(double nextTarget) {
    target = nextTarget;
    useVoltage = false;
  }

  /**
   * @param volts the target voltage
   * @apiNote Using this method causes {@link #atTarget()} to always return true!
   */
  public void setVoltage(double volts) {
    target = volts;
    useVoltage = true;
  }

  /**
   * Tell the motor what position it is actually at.
   *
   * @param actualPosition The position the motor actually is at.
   */
  public void resetPosition(double actualPosition) {
    positionSetter.accept(actualPosition);
  }

  /**
   * @return The current position of the motor
   */
  public double getPosition() {
    return positionGetter.getAsDouble();
  }

  /**
   * @return The current velocity of the motor
   */
  public double getVelocity() {
    return velocityGetter.getAsDouble();
  }

  /**
   * @apiNote This method returns true when using voltage control with {@link #setVoltage}
   * @return Whether we are about at the target specified by {@link #setTarget}
   */
  public boolean atTarget() {
    if (useVoltage) {
      return true;
    }
    switch (type) {
      case Position:
      case Rotation:
        fb.calculate(getPosition(), target);
        break;
      case Velocity:
        fb.calculate(getVelocity(), target);
        break;
    }
    return fb.atGoal();
  }

  /**
   * A method that is run every loop (20ms). This should never be called by the user
   *
   * @implNote calcuate the voltage to apply to the motor here
   */
  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      voltageSetter.accept(0);
      return;
    }
    if (useVoltage) {
      voltageSetter.accept(target);
      return;
    }
    double fbVolts = 0;
    double ffVolts = 0;
    switch (type) {
      case Position:
        fbVolts = fb.calculate(getPosition(), target);
        ffVolts = ff.kG() + ff.kS() * Math.signum(fbVolts);
        break;
      case Velocity:
        double velocity = getVelocity();
        fbVolts = fb.calculate(velocity, target);
        ffVolts = ff.kS() * Math.signum(target) + ff.kV() * target;
        break;
      case Rotation:
        double position = getPosition();
        fbVolts = fb.calculate(position, target);
        ffVolts = ff.kS() * Math.signum(fbVolts) + ff.kG() * Math.cos(2 * Math.PI * position);
        break;
    }
    voltageSetter.accept(fbVolts + ffVolts);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Motor Info", motorInfo);
    HoundLog.log(path, "Position", getPosition());
    HoundLog.log(path, "Velocity", getVelocity());
    HoundLog.log(path, "At Target", atTarget());
    HoundLog.log(path, "Target", target);
    if (useVoltage) {
      HoundLog.log(path, "Target Type", "Voltage");
    } else {
      HoundLog.log(path, "Target Type", type.name());
    }
  }

  /**
   * Gets a set of sysID commands to run to characterize a mechanism
   *
   * @param name Mechanism name
   * @param voltageRampRate The rate to increase volts at when running quasistatic tests, in
   *     volts/sec
   * @param stepVoltage The constant voltage to apply when running dynamic tests, in volts
   * @param duration How long the tests should last, in seconds
   * @return a set of SysIDCommands
   */
  public SysIDCommands getSysIDCommands(
      String name, double voltageRampRate, double stepVoltage, double duration) {
    Config config =
        new Config(
            Volts.of(voltageRampRate).per(Seconds), Volts.of(stepVoltage), Seconds.of(duration));
    Mechanism mech =
        new Mechanism(
            voltage -> setVoltage(voltage.in(Volts)),
            log ->
                log.motor("Motor")
                    .value("Position", getPosition(), "IDK")
                    .value("Velocity", getVelocity(), "IDK")
                    .value("Voltage", target, "Volts"),
            this,
            name);
    SysIdRoutine routine = new SysIdRoutine(config, mech);
    return new SysIDCommands(
        routine.dynamic(Direction.kForward),
        routine.dynamic(Direction.kReverse),
        routine.quasistatic(Direction.kForward),
        routine.quasistatic(Direction.kReverse));
  }

  /**
   * Very similar to {@link #getSysIDCommands getSysIDCommands} but for if you have multiple motors
   * in one mechanism that are linked
   *
   * <p>Examples: Shooter with two motors driving 1 axle, or a drivetrain with 4 motors driving one
   * robot
   *
   * @param name Mechanism name
   * @param voltageRampRate The rate to increase volts at when running quasistatic tests, in
   *     volts/sec
   * @param stepVoltage The constant voltage to apply when running dynamic tests, in volts
   * @param duration How long the tests should last, in seconds
   * @param otherMotors What other motors should also be synchronized for the test
   * @return The commands to run
   */
  public SysIDCommands getSynchronizedSysIDCommands(
      String name,
      double voltageRampRate,
      double stepVoltage,
      double duration,
      Motor... otherMotors) {
    Config config =
        new Config(
            Volts.of(voltageRampRate).per(Seconds), Volts.of(stepVoltage), Seconds.of(duration));
    Mechanism mech =
        new Mechanism(
            voltage -> {
              setVoltage(voltage.in(Volts));
              for (Motor motor : otherMotors) {
                motor.setVoltage(voltage.in(Volts));
              }
            },
            log -> {
              log.motor("Motor0")
                  .value("Position", getPosition(), "IDK")
                  .value("Velocity", getVelocity(), "IDK")
                  .value("Voltage", target, "Volts");
              for (int i = 0; i < otherMotors.length; i++) {
                Motor motor = otherMotors[i];
                log.motor("Motor" + (i + 1))
                    .value("Position", motor.getPosition(), "IDK")
                    .value("Velocity", motor.getVelocity(), "IDK")
                    .value("Voltage", motor.target, "Volts");
              }
            },
            this,
            name);
    SysIdRoutine routine = new SysIdRoutine(config, mech);
    SysIDCommands commands =
        new SysIDCommands(
            routine.dynamic(Direction.kForward),
            routine.dynamic(Direction.kReverse),
            routine.quasistatic(Direction.kForward),
            routine.quasistatic(Direction.kReverse));
    commands.dynamicForward().addRequirements(otherMotors);
    commands.dynamicReverse().addRequirements(otherMotors);
    commands.quasistaticForward().addRequirements(otherMotors);
    commands.quasistaticReverse().addRequirements(otherMotors);
    return commands;
  }

  /** The types of targets a motor can have as its goal */
  public static enum TargetType {
    /** Targets a position */
    Position,
    /** Targets a velocity */
    Velocity,
    /** Targets a vertical rotation, taking into account the changing gravitational force */
    Rotation;
  }

  /**
   * An object that holds feedforward gains
   *
   * <p>{@link #getSysIDCommands} can be used to obtain values for gains
   *
   * @param kG The voltage needed to hold the mechanism in place against gravity. For most
   *     mechanisms, this is a constant value, but for mechanisms rotating vertically, this should
   *     be the voltage to hold it up parallel to the ground
   * @param kS The voltage needed to overcome static friction.
   * @param kV The voltage needed to maintain a velocity of 1 unit/s
   * @param kA The voltage needed to induce an acceleration of 1 unit/s^
   * @see <a href =
   *     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#introduction-to-dc-motor-feedforward">
   *     Introduction to Feedforward
   */
  public static record FeedforwardConstants(double kG, double kS, double kV, double kA) {}

  /**
   *
   *
   * <pre>
   * // Example
   * Motor motor = Motor.fromTalonFX( // Make a TalonFX motor
   *   7, // The motor's CAN id  is 7
   *   fx -> {
   *     TalonFXConfiguration config = new TalonFXConfiguration(); // Make a new TalonFX config
   *     config.CurrentLimits =
   *       new CurrentLimitsConfigs()
   *         .withSupplyCurrentLimit(40) // This motor has a supply current limit of 40 amps
   *         .withSupplyCurrentLimitEnable(true); // Enable the limit
   *     config.MotorOutput =
   *       new MotorOutputConfigs()
   *         .withNeutralMode(NeutralModeValue.Brake) // If zero voltage, the motor will brake
   *         .withInterted(InvertedValue.CounterClockwise_Positive); // The positive direction is CCW
   *     config.Feedback =
   *       new FeedbackConfigs()
   *         .withSensorToMechanismRatio(12.1908); // 12.1908 Mechanism Units : 1 Sensor Unit
   *     motor.getConfigurator().apply(config); // Apply the config
   *   },
   *   sim -> {
   *     sim.withHardstops(0, 30); // The mechanism has hardstops at 0 and 30 units
   *   },
   *   0, // The starting position of the motor is 0 units
   *   FeedbackController.fromPID( // Using PID for our feedback
   *     new PIDController(5, 0, 0), // Our PID values
   *     pid -> { // Configuring the pid controller
   *       pid.setTolerance(1); // Within one unit to our goal is good enough
   *     }
   *   ),
   *   new FeedforwardConstants(0, 0.10624, 1.407, 0.16994), // Our feedforward values
   *   TargetType.Position // This motor goes to a position
   * );
   * </pre>
   *
   * @param canID The canID of the controller
   * @param config A function that takes in a {@link TalonFX} and configures it
   * @param simConfig A function that takes in a {@link FeedforwardSim} and configures it
   * @param initialPosition The starting position of the mechanism. This is set after the config has
   *     been applied, so any gear ratios applied in the config are used
   * @param fb The feedback controller used
   * @param ff The feedforward controller used. This can be null!!
   * @param type The type of target the motor is trying to reach
   * @return A TalonFX Motor Controller wrapped in a {@link Motor}.
   *     <p>In Simulation, either {@link #fromIdealSim} or {@link #fromRealisticSim} is returned,
   *     depending on whether ff is null
   */
  public static Motor fromTalonFX(
      int canID,
      Consumer<TalonFX> config,
      Consumer<FeedforwardSim> simConfig,
      double initialPosition,
      FeedbackController fb,
      Optional<FeedforwardConstants> ff,
      TargetType type) {
    if (RobotBase.isSimulation()) {
      if (ff.isEmpty()) {
        return fromIdealSim(fb, type, initialPosition);
      } else {
        return fromRealisticSim(simConfig, fb, ff.get(), type, initialPosition);
      }
    }
    TalonFX motor = new TalonFX(canID);
    config.accept(motor);
    motor.setPosition(initialPosition);
    return new Motor(
        type,
        motor::setPosition,
        motor::setVoltage,
        () -> motor.getPosition().getValueAsDouble(),
        () -> motor.getVelocity().getValueAsDouble(),
        fb,
        ff,
        path -> {
          HoundLog.log(path, "Acceleration", motor.getAcceleration().getValueAsDouble());
          HoundLog.log(path, "Temperature", motor.getDeviceTemp().getValueAsDouble());
          HoundLog.log(path, "Stator Current", motor.getStatorCurrent().getValueAsDouble());
          HoundLog.log(path, "Supply Current", motor.getSupplyCurrent().getValueAsDouble());
          HoundLog.log(path, "Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
          HoundLog.log(path, "Bus Voltage", motor.getSupplyVoltage().getValueAsDouble());
          HoundLog.log(path, "Motor Status", motor.getMotorOutputStatus().getValue());
        });
  }

  /**
   *
   *
   * <pre>
   * // Example
   * Motor motor = Motor.fromSparkMax( // Make a SparkMax motor
   *   7, // The motor's CAN id  is 7
   *   spark -> {
   *     SparkMaxConfig config = new SparkMaxConfig(); // Make a new SparkMaxConfig
   *     config
   *       .inverted(false) // The motor isn't inverted
   *       .smartCurrentLimit(20) // It has a supply current limit of 20
   *       .idleMode(kBrake) // When there is no voltage, the motor will brake
   *     config
   *       .encoder
   *       .positionConversionFactor(1.0 / 25) // One mechanism unit : 25 sensor units
   *       .velocityConversionFactor(1.0 / 25/ 60) // Divide by 60 so rpm -> rps
   *     spark.configure(config, kResetSafeParameters, kPersistParameters)
   *   },
   *   sim -> {
   *     sim.withHardstops(0, 30); // The mechanism has hardstops at 0 and 30 units
   *   },
   *   0, // The starting position of the motor is 0 units
   *   FeedbackController.fromPID( // Using PID for our feedback
   *     new PIDController(5, 0, 0), // Our PID values
   *     pid -> { // Configuring the pid controller
   *       pid.setTolerance(1); // Within one unit to our goal is good enough
   *     }
   *   ),
   *   new FeedforwardConstants(0, 0.10624, 1.407, 0.16994), // Our feedforward values
   *   TargetType.Position // This motor goes to a position
   * );
   * </pre>
   *
   * @param canID The canID of the controller
   * @param brushed Whether the motor controlled by the controller is brushed or not. This is
   *     probably false!!
   * @param config A function that takes in a {@link SparkMax} and configures it
   * @param simConfig A function that takes in a {@link FeedforwardSim} and configures it
   * @param initialPosition The starting position of the mechanism. This is set after the config has
   *     been applied, so any gear ratios applied in the config are used
   * @param fb The feedback controller used
   * @param ff The feedforward controller used. This can be null!!
   * @param type The type of target the motor is trying to reach
   * @return A SparkMax Motor Controller wrapped in a {@link Motor}.
   *     <p>In Simulation, either {@link #fromIdealSim} or {@link #fromRealisticSim} is returned,
   *     depending on whether ff is null
   */
  public static Motor fromSparkMax(
      int canID,
      boolean brushed,
      Consumer<SparkMax> config,
      Consumer<FeedforwardSim> simConfig,
      double initialPosition,
      FeedbackController fb,
      Optional<FeedforwardConstants> ff,
      TargetType type) {
    if (RobotBase.isSimulation()) {
      if (ff.isEmpty()) {
        return fromIdealSim(fb, type, initialPosition);
      } else {
        return fromRealisticSim(simConfig, fb, ff.get(), type, initialPosition);
      }
    }
    SparkMax motor = new SparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
    config.accept(motor);
    motor.getEncoder().setPosition(initialPosition);
    return new Motor(
        type,
        position -> motor.getEncoder().setPosition(position),
        motor::setVoltage,
        () -> motor.getEncoder().getPosition(),
        () -> motor.getEncoder().getVelocity(),
        fb,
        ff,
        path -> {
          HoundLog.log(path, "Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
          HoundLog.log(path, "Temperature", motor.getMotorTemperature());
          HoundLog.log(path, "Stator Current", motor.getOutputCurrent());
          HoundLog.log(path, "Bus Voltage", motor.getBusVoltage());
        });
  }

  /**
   *
   *
   * <pre>
   * // Example
   * Motor motor = Motor.fromTalonSRX( // Make a TalonSRX motor
   *   7,
   *   srx -> {
   *     srx.configSupplyCurrentLimit( // Set a supply current limit of 30.
   *       // Limit is triggered if current > 40 for over 0.1 seconds
   *       new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1),
   *       0 // Don't wait for confirmation from the controller
   *     );
   *   },
   *   sim -> {
   *     sim.withHardstops(0, 30); // The mechanism has hardstops at 0 and 30 units
   *   },
   *   1.0 / 1000, // One mechanism unit is equal to 1000 sensor units
   *   0, // The starting position of the motor is 0 units
   *   FeedbackController.fromPID( // Using PID for our feedback
   *     new PIDController(5, 0, 0), // Our PID values
   *     pid -> { // Configuring the pid controller
   *       pid.setTolerance(1); // Within one unit to our goal is good enough
   *     }
   *   ),
   *   new FeedforwardConstants(0, 0.10624, 1.407, 0.16994), // Our feedforward values
   *   TargetType.Position // This motor goes to a position
   * );
   * </pre>
   *
   * @param canID The canID of the controller
   * @param config A function that takes in a {@link TalonSRX} and configures it
   * @param simConfig A function that takes in a {@link FeedforwardSim} and configures it
   * @param conversionFactor The gear ratio before the encoder, in mechanism units per sensor unit
   * @param initialPosition The starting position of the mechanism
   * @param fb The feedback controller used
   * @param ff The feedforward controller used
   * @param type The type of target the motor is trying to reach
   * @return A TalonSRX Motor Controller wrapped in a {@link Motor}
   *     <p>In Simulation, either {@link #fromIdealSim} or {@link #fromRealisticSim} is returned,
   *     depending on whether ff is present
   */
  public static Motor fromTalonSRX(
      int canID,
      Consumer<TalonSRX> config,
      Consumer<FeedforwardSim> simConfig,
      double conversionFactor,
      double initialPosition,
      FeedbackController fb,
      Optional<FeedforwardConstants> ff,
      TargetType type) {
    if (RobotBase.isSimulation()) {
      if (ff.isEmpty()) {
        return fromIdealSim(fb, type, initialPosition);
      } else {
        return fromRealisticSim(simConfig, fb, ff.get(), type, initialPosition);
      }
    }
    TalonSRX motor = new TalonSRX(canID);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1), 0);
    config.accept(motor);
    motor.setSelectedSensorPosition(initialPosition / conversionFactor);
    return new Motor(
        type,
        position -> motor.setSelectedSensorPosition(position / conversionFactor),
        voltage -> motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage()),
        () -> motor.getSelectedSensorPosition() * conversionFactor,
        () -> motor.getSelectedSensorVelocity() * 10 * conversionFactor,
        fb,
        ff,
        path -> {
          HoundLog.log(path, "Bus Voltage", motor.getBusVoltage());
          HoundLog.log(path, "Temperature", motor.getTemperature());
          HoundLog.log(path, "Output Voltage", motor.getMotorOutputVoltage());
          HoundLog.log(path, "Stator Current", motor.getStatorCurrent());
          HoundLog.log(path, "Supply Current", motor.getSupplyCurrent());
        });
  }

  /**
   *
   *
   * <pre>
   * // Example
   * Motor motor = Motor.fromRealisticSim( // Make a realistic sim
   *   sim -> {
   *     sim.withHardstops(0, 30); // The mechanism has hardstops at 0 and 30 units
   *   },
   *   FeedbackController.fromPID( // Using PID for our feedback
   *     new PIDController(5, 0, 0), // Our PID values
   *     pid -> { // Configuring the pid controller
   *       pid.setTolerance(1); // Within one unit to our goal is good enough
   *     }
   *   ),
   *   new FeedforwardConstants(0, 0.10624, 1.407, 0.16994), // Our feedforward values
   *   TargetType.Position, // This motor goes to a position
   *   0 // The starting position of the motor is 0 units
   * );
   * </pre>
   *
   * @param config A function that takes in a {@link FeedforwardSim} and configures it
   * @param fb The feedback controller used
   * @param ff The feedforward controller used
   *     <p><strong>If kV or kA is 0, or ff is null, {@link #fromIdealSim} will be used
   *     instead</strong>
   * @param type The type of target the motor is trying to reach
   * @param initialPosition The starting position of the mechanism
   * @return A simulated {@link Motor}, using the constants from ff to predict movement
   */
  public static Motor fromRealisticSim(
      Consumer<FeedforwardSim> config,
      FeedbackController fb,
      FeedforwardConstants ff,
      TargetType type,
      double inititalPosition) {
    if (ff == null || ff.kV() == 0 || ff.kA() == 0) {
      return fromIdealSim(fb, type, inititalPosition);
    }
    FeedforwardSim sim = new FeedforwardSim(ff, inititalPosition, type == TargetType.Rotation);
    config.accept(sim);
    return new Motor(
        type,
        sim::resetPosition,
        sim::setVoltage,
        () -> sim.getState().position(),
        () -> sim.getState().velocity(),
        fb,
        Optional.of(ff),
        path -> {
          HoundLog.log(path, "Sim", sim);
        });
  }

  /**
   *
   *
   * <pre>
   * // Example
   * Motor motor = Motor.fromIdealSim( // Make an ideal sim
   *   FeedbackController.fromPID( // Using PID for our feedback
   *     new PIDController(5, 0, 0), // Our PID values
   *     pid -> { // Configuring the pid controller
   *       pid.setTolerance(1); // Within one unit to our goal is good enough
   *     }
   *   ),
   *   TargetType.Position, // This motor goes to a position
   *   0 // The starting position of the motor is 0 units
   * );
   * </pre>
   *
   * @param fb The feedback controller used
   * @param type The type of target the motor is trying to reach
   * @param initialPosition The starting position of the mechanism
   * @return A {@link Motor} that teleports to the setpoint of fb. Note that something somewhat
   *     realistic can be cobbled together if fb is wrapping a ProfiledPIDController
   */
  public static Motor fromIdealSim(FeedbackController fb, TargetType type, double initialPosition) {
    /*
      In lambdas, variables must be "effectivly final", so the state of the motor is
      stored in this double array, where element 0 is position, element 1 is velocity,
      and element 2 is acceleration
    */
    double[] stateHolder = new double[] {initialPosition, 0, 0};
    return new Motor(
        type,
        position -> stateHolder[0] = position,
        voltage -> {
          if (voltage == 0) {
            stateHolder[1] = 0;
            stateHolder[2] = 0;
            return;
          }
          switch (type) {
            case Velocity:
              State nextStateVel = fb.getSetpoint();
              stateHolder[1] = nextStateVel.position;
              stateHolder[2] = nextStateVel.velocity;
              stateHolder[0] += 0.02 * stateHolder[1];
              break;
            default:
              State nextState = fb.getSetpoint();
              stateHolder[2] = (nextState.velocity - stateHolder[1]) / 0.02;
              stateHolder[0] = nextState.position;
              stateHolder[1] = nextState.velocity;
              break;
          }
        },
        () -> stateHolder[0],
        () -> stateHolder[1],
        fb,
        null,
        path -> HoundLog.log(path, "Acceleration", stateHolder[2]));
  }
}
