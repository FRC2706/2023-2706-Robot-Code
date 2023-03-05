package frc.robot;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfileExternalPIDController {

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints m_constraints;

    private double m_period;

    public ProfileExternalPIDController(TrapezoidProfile.Constraints constraints) {
        this(constraints, 0.02);
      }

    public ProfileExternalPIDController(TrapezoidProfile.Constraints constraints, double period) {
        m_constraints = constraints;
        m_period = period;
    }
    
      /**
       * Sets the goal for the ProfiledExternalPIDController.
       *
       * @param goal The desired goal state.
       */
      public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
      }
    
      /**
       * Sets the goal for the ProfiledExternalPIDController.
       *
       * @param goal The desired goal position.
       */
      public void setGoal(double goal) {
        m_goal = new TrapezoidProfile.State(goal, 0);
      }
    
      /**
       * Gets the goal for the ProfiledExternalPIDController.
       *
       * @return The goal.
       */
      public TrapezoidProfile.State getGoal() {
        return m_goal;
      }
    
      /**
       * Set velocity and acceleration constraints for goal.
       *
       * @param constraints Velocity and acceleration constraints for goal.
       */
      public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
      }
    
      /**
       * Returns the current setpoint of the ProfiledExternalPIDController.
       *
       * @return The current setpoint.
       */
      public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
      }
    
      /**
       * Returns the next output of the PID controller.
       *
       * @param measurement The current measurement of the process variable.
       * @return The controller's next output.
       */
      public double getPIDSetpoint() {
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(m_period);
        return m_setpoint.position;
      }
    
      /**
       * Returns the next output of the PID controller.
       *
       * @param measurement The current measurement of the process variable.
       * @param goal The new goal of the controller.
       * @return The controller's next output.
       */
      public double getPIDSetpoint(TrapezoidProfile.State goal) {
        setGoal(goal);
        return getPIDSetpoint();
      }
    
      /**
       * Returns the next output of the PIDController.
       *
       * @param measurement The current measurement of the process variable.
       * @param goal The new goal of the controller.
       * @return The controller's next output.
       */
      public double getPIDSetpoint( double goal) {
        setGoal(goal);
        return getPIDSetpoint();
      }
    
      /**
       * Returns the next output of the PID controller.
       *
       * @param measurement The current measurement of the process variable.
       * @param goal The new goal of the controller.
       * @param constraints Velocity and acceleration constraints for goal.
       * @return The controller's next output.
       */
      public double getPIDSetpoint(TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return getPIDSetpoint(goal);
      }
    
      /**
       * Reset the previous error and the integral term.
       *
       * @param measurement The current measured State of the system.
       */
      public void reset(TrapezoidProfile.State measurement) {
        m_setpoint = measurement;
      }
    
      /**
       * Reset the previous error and the integral term.
       *
       * @param measuredPosition The current measured position of the system.
       * @param measuredVelocity The current measured velocity of the system.
       */
      public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
      }
    
      /**
       * Reset the previous error and the integral term.
       *
       * @param measuredPosition The current measured position of the system. The velocity is assumed to
       *     be zero.
       */
      public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
      }
    
}
