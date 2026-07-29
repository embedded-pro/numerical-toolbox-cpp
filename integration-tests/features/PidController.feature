Feature: Incremental PID controller

    The synchronous incremental PID controller drives its control output so that
    the measured process variable approaches the configured set point, while
    never exceeding the configured output limits.

    Scenario: Output drives the process variable towards the set point
        Given a PID controller with gains kp 1.0 ki 0.5 kd 0.1
        And output limits between -100.0 and 100.0
        And a set point of 10.0
        When a measurement of 0.0 is processed
        Then the control output should be greater than 0.0

    Scenario: Output is clamped to the configured limits
        Given a PID controller with gains kp 50.0 ki 10.0 kd 0.0
        And output limits between -5.0 and 5.0
        And a set point of 100.0
        When a measurement of 0.0 is processed
        Then the control output should be 5.0
