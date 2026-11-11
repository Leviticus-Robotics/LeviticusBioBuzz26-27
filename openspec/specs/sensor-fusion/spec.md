# Sensor Fusion Specification

## Purpose

Provide a timestamp-aware probabilistic localization capability that combines predicted motion, relative odometry, absolute field observations, and future sensor factors into one authoritative robot pose-and-velocity estimate.

## Requirements

### Requirement: Authoritative Gaussian robot state
The system SHALL maintain one authoritative Gaussian estimate of field position, wrapped heading, field-relative translational velocity, and angular velocity, including covariance between all state dimensions.

#### Scenario: Fused state is available
- **WHEN** localization has been initialized and advanced to a requested timestamp
- **THEN** the system returns the estimated pose, velocity, and their covariance for that timestamp

#### Scenario: Heading crosses the wrap boundary
- **WHEN** a prediction or correction crosses the positive or negative pi boundary
- **THEN** the system normalizes heading and computes the shortest signed angular residual

### Requirement: Localization-owned planar pose type
The localization package SHALL represent planar poses with its own immutable, sensor- and library-independent pose type. Concrete localization models and factors SHALL use that type at the localization boundary, while the generic inference engine and generic factor contracts SHALL operate on dimension-independent Gaussians. Neither boundary SHALL expose Pedro, OctoQuad, FTC Vision, or other sensor-specific pose types.

#### Scenario: Sensor reading enters fusion
- **WHEN** an OctoQuad, AprilTag, or future sensor adapter creates a pose measurement or relative-pose measurement
- **THEN** the adapter converts the reading into the localization-owned pose type and creates a localization factor backed by the generic Gaussian contracts

#### Scenario: Pedro reads fused localization
- **WHEN** the Pedro localizer adapter requests the latest fused pose
- **THEN** the adapter converts the localization-owned pose into Pedro's pose representation without exposing Pedro types to the inference engine or factor contracts

#### Scenario: Localization consumer is replaced
- **WHEN** Pedro is replaced by another localization consumer
- **THEN** the inference engine, localization state, and factor contracts require no changes and only the consumer adapter is replaced

### Requirement: Covariance and information representations
The system SHALL support equivalent covariance-form and information-form Gaussian representations and SHALL preserve the represented mean and covariance across valid round trips between them.

#### Scenario: Gaussian representation round trip
- **WHEN** a valid positive-definite Gaussian is converted to information form and back
- **THEN** the resulting mean and covariance match the original within configured numerical tolerance

#### Scenario: Invalid uncertainty input
- **WHEN** a factor provides a covariance with incompatible dimensions, non-finite values, or an unusable information matrix
- **THEN** the system rejects the factor without corrupting the current estimate and reports a diagnostic reason

### Requirement: Timestamped process prediction
The system SHALL advance the state from one timestamp to another using the process model, the control input active over each interval, and process covariance scaled for that interval. Each elapsed interval SHALL be predicted exactly once.

#### Scenario: Control input changes
- **WHEN** a new control input becomes active at timestamp `t`
- **THEN** the system first predicts through `t` using the previous input and uses the new input only for intervals after `t`

#### Scenario: Multiple factors share a timestamp
- **WHEN** multiple factors are applied at the same timestamp
- **THEN** the system performs no additional process prediction between those factors

#### Scenario: Prediction without sensor updates
- **WHEN** no sensor factors arrive during an elapsed interval
- **THEN** the system continues to predict the state and increases uncertainty according to the process model

### Requirement: Sensor-independent factor submission
The system SHALL accept timestamped factors through sensor-independent absolute and relative interfaces. Standard measurement factors SHALL provide a Gaussian measurement and `stateToMeasurement` mapping, while standard transition factors SHALL provide a covariance-aware state transition.

#### Scenario: New compatible sensor factor
- **WHEN** a new sensor adapter produces a valid factor using the supported state and timing contract
- **THEN** the inference engine incorporates it without requiring sensor-specific logic in the engine

### Requirement: Relative OctoQuad odometry
The system SHALL derive an OctoQuad relative-pose factor from consecutive OctoQuad poses and SHALL use that factor to constrain robot motion between the corresponding timestamps. The system SHALL NOT treat each integrated OctoQuad pose as an independent absolute field observation.

#### Scenario: Consecutive OctoQuad readings
- **WHEN** valid OctoQuad poses are received at consecutive timestamps
- **THEN** the system computes their relative SE(2) transform and constrains the fused state transition using the configured delta covariance

#### Scenario: First OctoQuad reading
- **WHEN** the first valid OctoQuad pose is received
- **THEN** the system stores it as the relative-motion baseline without creating a false zero-origin absolute measurement

#### Scenario: Absolute correction followed by odometry
- **WHEN** an absolute field observation corrects the fused pose and a later OctoQuad delta arrives
- **THEN** the new delta advances from the corrected fused trajectory rather than pulling the estimate back toward the OctoQuad's prior global frame

#### Scenario: OctoQuad-only operation
- **WHEN** OctoQuad factors are available, absolute factors are absent, and OctoQuad uncertainty dominates the process-model uncertainty
- **THEN** the fused pose follows the accumulated OctoQuad relative motion and reproduces standard odometry behavior within numerical tolerance

### Requirement: No duplicate OctoQuad-derived information
The initial OctoQuad adapter SHALL use relative pose change as its measurement and SHALL NOT independently add the OctoQuad's derived velocity or built-in IMU heading as additional factors from the same underlying sample.

#### Scenario: OctoQuad factor creation
- **WHEN** an OctoQuad sample contains integrated pose and derived velocity
- **THEN** the initial adapter emits one relative-pose factor and does not emit a second velocity or IMU factor from that sample

### Requirement: Absolute field-pose observations
The system SHALL accept absolute field-pose factors with configured covariance and SHALL correct the fused state according to both predicted and measurement uncertainty.

#### Scenario: Valid absolute pose factor
- **WHEN** a valid absolute pose factor is submitted at a supported timestamp
- **THEN** the posterior moves toward the measured field pose by the amount determined by the factor and predicted covariances

#### Scenario: Moving or unknown field reference
- **WHEN** a detection does not have a known stationary field pose
- **THEN** the system excludes it from absolute localization

### Requirement: Same-frame AprilTag fusion
The system SHALL combine usable AprilTag pose estimates from one camera frame into one camera factor using normalized information so the frame's total information does not increase solely with detection count.

#### Scenario: Multiple agreeing tags in one frame
- **WHEN** multiple usable tag estimates share one acquisition timestamp and pass validation
- **THEN** the system produces one weighted camera pose whose information is the average of the accepted per-tag information contributions

#### Scenario: Tag count changes with identical estimates
- **WHEN** identical per-tag Gaussian estimates are duplicated within one frame
- **THEN** the combined mean and covariance remain unchanged

#### Scenario: Same-frame outlier
- **WHEN** one tag estimate is inconsistent with the other accepted estimates beyond the configured gate
- **THEN** the system excludes or downweights that estimate and reports the decision in diagnostics

### Requirement: Delayed observation handling
The system SHALL preserve a bounded history sufficient to apply a delayed factor at its measurement timestamp and propagate the resulting correction to the latest estimate.

#### Scenario: Delayed AprilTag frame within history
- **WHEN** an AprilTag factor arrives after its capture time but within retained history
- **THEN** the system applies it at the capture time and recomputes affected later transitions and factors

#### Scenario: Factor older than retained history
- **WHEN** a factor timestamp predates the retained history
- **THEN** the system rejects it or applies an explicitly configured degraded policy and reports which policy was used

### Requirement: Factor validation and diagnostics
The system SHALL validate factor timestamps, dimensions, finite values, and covariance validity before applying them, and SHALL expose diagnostics for accepted and rejected factors, retained history, full-history recomputations, process-model evaluations, active input, and latest accepted source timestamps. Statistical outlier handling MAY be performed by a sensor adapter, but SHALL NOT be required by the core Gaussian fusion operation.

#### Scenario: Invalid factor
- **WHEN** a factor has invalid timing, incompatible dimensions, non-finite values, or unusable covariance
- **THEN** the system rejects the factor without altering the estimate and records its source and rejection reason

#### Scenario: Fusion telemetry
- **WHEN** localization telemetry is requested
- **THEN** the system reports the current pose, velocity, uncertainty summary, active input, latest factor times, and factor acceptance or rejection counts

### Requirement: Pedro localization compatibility
The system SHALL expose the latest fused pose, velocity, and twist through Pedro's `Localizer` contract and SHALL support explicit pose initialization or reset without using sensor-specific fusion logic in the Pedro adapter.

#### Scenario: Follower reads localization
- **WHEN** Pedro requests the current localization state
- **THEN** it receives the latest fused pose and motion values in the configured units and coordinate conventions

#### Scenario: Pose initialization
- **WHEN** autonomous or teleoperation code sets an initial field pose
- **THEN** the fused state is initialized consistently while future OctoQuad readings continue to contribute relative motion
