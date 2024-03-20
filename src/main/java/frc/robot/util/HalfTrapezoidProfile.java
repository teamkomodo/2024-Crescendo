// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Adapted by Team 4293

package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import java.util.Objects;

public class HalfTrapezoidProfile {
  // The direction of the profile, either 1 for forwards or -1 for inverted
  private int m_direction;

  private final Constraints m_constraints;
  private State m_current;

  private double m_endFullSpeed;
  private double m_endDeccel;

  /** Profile constraints. */
  public static class Constraints {
    /** Maximum velocity. */
    public final double maxVelocity;

    /** Maximum acceleration. */
    public final double maxDecceleration;

    /**
     * Construct constraints for a TrapezoidProfile.
     *
     * @param maxVelocity maximum velocity
     * @param maxDecceleration maximum acceleration
     */
    public Constraints(double maxVelocity, double maxDecceleration) {
      this.maxVelocity = maxVelocity;
      this.maxDecceleration = maxDecceleration;
      MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);
    }

    /**
     * Construct constraints for a TrapezoidProfile.
     *
     * @param <U> Unit type.
     * @param maxVelocity maximum velocity
     * @param maxDecceleration maximum acceleration
     */
    public <U extends Unit<U>> Constraints(
        Measure<Velocity<U>> maxVelocity, Measure<Velocity<Velocity<U>>> maxDecceleration) {
      this(maxVelocity.baseUnitMagnitude(), maxDecceleration.baseUnitMagnitude());
    }
  }

  /** Profile state. */
  public static class State {
    /** The position at this state. */
    public double position;

    /** The velocity at this state. */
    public double velocity;

    /** Default constructor. */
    public State() {}

    /**
     * Constructs constraints for a Trapezoid Profile.
     *
     * @param position The position at this state.
     * @param velocity The velocity at this state.
     */
    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    /**
     * Constructs constraints for a Trapezoid Profile.
     *
     * @param <U> Unit type.
     * @param position The position at this state.
     * @param velocity The velocity at this state.
     */
    public <U extends Unit<U>> State(Measure<U> position, Measure<Velocity<U>> velocity) {
      this(position.baseUnitMagnitude(), velocity.baseUnitMagnitude());
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        return this.position == rhs.position && this.velocity == rhs.velocity;
      } else {
        return false;
      }
    }

    @Override
    public int hashCode() {
      return Objects.hash(position, velocity);
    }
  }

  /**
   * Construct a TrapezoidProfile.
   *
   * @param constraints The constraints on the profile, like maximum velocity.
   */
  public HalfTrapezoidProfile(Constraints constraints) {
    m_constraints = constraints;
  }

  /**
   * Calculate the correct position and velocity for the profile at a time t where the beginning of
   * the profile was at time t = 0.
   *
   * @param t The time since the beginning of the profile.
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @return The position and velocity of the profile at time t.
   */
  public State calculate(double t, State current, State goal) {
    m_direction = shouldFlipDecceleration(current, goal) ? -1 : 1;
    m_current = direct(current);
    goal = direct(goal);

    if (m_current.velocity > m_constraints.maxVelocity) {
      m_current.velocity = m_constraints.maxVelocity;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = m_current.velocity / m_constraints.maxDecceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxDecceleration / 2.0;

    double cutoffEnd = goal.velocity / m_constraints.maxDecceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxDecceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    double fullTrapezoidDist =
        cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
    double accelerationTime = m_constraints.maxVelocity / m_constraints.maxDecceleration;

    double fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxDecceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxDecceleration);
      fullSpeedDist = 0;
    }

    m_endFullSpeed = fullSpeedDist / m_constraints.maxVelocity;
    m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
    State result = new State(m_current.position, m_current.velocity);

    if (t < m_endFullSpeed) {
      result.velocity = m_constraints.maxVelocity;
      result.position += m_constraints.maxVelocity * (t);
    } else if (t <= m_endDeccel) {
      result.velocity = goal.velocity + (m_endDeccel - t) * m_constraints.maxDecceleration;
      double timeLeft = m_endDeccel - t;
      result.position =
          goal.position
              - (goal.velocity + timeLeft * m_constraints.maxDecceleration / 2.0) * timeLeft;
    } else {
      result = goal;
    }

    return direct(result);
  }

  /**
   * Returns the total time the profile takes to reach the goal.
   *
   * @return The total time the profile takes to reach the goal.
   */
  public double totalTime() {
    return m_endDeccel;
  }

  /**
   * Returns true if the profile has reached the goal.
   *
   * <p>The profile has reached the goal if the time since the profile started has exceeded the
   * profile's total time.
   *
   * @param t The time since the beginning of the profile.
   * @return True if the profile has reached the goal.
   */
  public boolean isFinished(double t) {
    return t >= totalTime();
  }

  /**
   * Returns true if the profile inverted.
   *
   * <p>The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the current state).
   * @param goal The desired state when the profile is complete.
   */
  private static boolean shouldFlipDecceleration(State initial, State goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  private State direct(State in) {
    State result = new State(in.position, in.velocity);
    result.position = result.position * m_direction;
    result.velocity = result.velocity * m_direction;
    return result;
  }
}

