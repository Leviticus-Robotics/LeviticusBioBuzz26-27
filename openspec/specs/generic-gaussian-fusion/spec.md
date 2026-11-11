# Generic Gaussian Fusion Specification

## Purpose

Provide reusable, domain-independent Gaussian prediction and factor inference so localization and non-pose robot subsystems can share one validated probabilistic framework.

## Requirements

### Requirement: Dimension-independent Gaussian state
The system SHALL represent an estimated state as a Gaussian of any positive dimension and SHALL NOT require pose, heading, velocity, localization, robotics, or sensor-specific semantics in the generic fusion capability.

#### Scenario: Non-pose subsystem state
- **WHEN** a subsystem initializes a two-dimensional state containing mechanism position and velocity
- **THEN** the fusion capability accepts, predicts, measures, and returns that state without localization-specific configuration or types

#### Scenario: State dimension mismatch
- **WHEN** an operation receives a mean, covariance, information matrix, state-to-measurement mapping, or factor whose dimensions are incompatible
- **THEN** the system rejects the operation without mutating the authoritative estimate and reports a diagnostic reason

### Requirement: Equivalent covariance and information representations
The system SHALL support covariance-form and information-form representations of the same multivariate Gaussian and SHALL preserve the represented distribution across valid conversions.

#### Scenario: Representation round trip
- **WHEN** a positive-definite Gaussian is converted from covariance form to information form and back
- **THEN** the resulting mean and covariance equal the original values within configured numerical tolerance

### Requirement: Generic Gaussian measurement fusion
The system SHALL fuse a Gaussian state with a Gaussian measurement of any compatible linear state projection using the supplied `stateToMeasurement` mapping and measurement covariance.

#### Scenario: Partial-state observation
- **WHEN** a factor measures one component of a larger state through a compatible `stateToMeasurement` mapping
- **THEN** the posterior incorporates that observation and updates other dimensions only according to their represented covariance with the observed component

#### Scenario: Different measurement dimensions
- **WHEN** compatible factors observe one, several, or all state dimensions
- **THEN** the same fusion operation incorporates each factor without domain-specific branches in the inference engine

### Requirement: Absolute and relative factor categories
The system SHALL accept absolute factors that apply to the current state and relative factors that use a retained state from an earlier timestamp. Gaussian measurement factors SHALL be supported as absolute factors, and measured state transitions SHALL be supported as relative factors.

#### Scenario: Absolute measurement factor
- **WHEN** an absolute factor supplies a valid Gaussian measurement for the current timestamp
- **THEN** the engine applies it to the current predicted Gaussian without requiring knowledge of the producing sensor or subsystem

#### Scenario: Relative transition factor
- **WHEN** a relative factor supplies a valid measured transition from a retained earlier timestamp
- **THEN** the engine applies the transition to that retained Gaussian and propagates its uncertainty to the factor timestamp

#### Scenario: Unsupported relative history
- **WHEN** a relative factor references a timestamp older than retained history
- **THEN** the engine rejects the factor without changing the estimate and reports that the referenced state is unavailable

### Requirement: Generic process prediction
The system SHALL advance a Gaussian state with a caller-supplied process model whose input type is selected by the consuming domain. The generic fusion capability SHALL NOT interpret the input or assume particular state dimensions.

#### Scenario: Domain-selected process input
- **WHEN** a consumer supplies a process model and timestamped input value
- **THEN** the engine delegates state and covariance prediction to that model using the input active over each elapsed interval

#### Scenario: Input changes between predictions
- **WHEN** the active input changes at timestamp `t`
- **THEN** the engine predicts through `t` with the previous input and uses the new input only after `t`

### Requirement: Incremental inference with bounded delayed-factor recomputation
The system SHALL apply chronological events incrementally, maintain bounded timestamped history sufficient to insert supported delayed factors, recompute subsequent inputs and factors deterministically when required, and return the latest corrected Gaussian.

#### Scenario: Chronological event fast path
- **WHEN** a new input, prediction tick, or factor is later than the current estimate
- **THEN** the engine advances and applies it incrementally without recomputing earlier retained events

#### Scenario: Delayed factor within history
- **WHEN** a valid factor arrives with a timestamp inside retained history
- **THEN** the engine applies it at that timestamp and deterministically recomputes the affected later estimate

#### Scenario: History remains bounded
- **WHEN** retained events exceed configured time or event-count limits
- **THEN** the engine checkpoints the retained estimate and removes older events without changing the latest represented state

### Requirement: Domain dependency isolation
The generic fusion capability SHALL have no compile-time dependency on localization geometry, robot-state wrappers, Pedro, FTC hardware or vision APIs, AprilTags, OctoQuad, or other concrete subsystem types.

#### Scenario: Architecture dependency check
- **WHEN** the generic fusion source package is inspected or compiled independently of localization sources
- **THEN** no import or public signature references a localization, Pedro, FTC sensor, or hardware type

#### Scenario: Localization consumes generic fusion
- **WHEN** robot localization performs process prediction, absolute pose fusion, relative odometry transition, delayed-data recomputation, or diagnostics
- **THEN** localization supplies its pose-specific models and factors through the generic contracts while preserving observable pose, velocity, covariance, timing, and telemetry behavior, and a state reset clears event history while preserving the active process input

### Requirement: Atomic validation and diagnostics
The system SHALL validate timestamps, dimensions, finite values, covariance validity, and factor compatibility before committing a factor, and SHALL expose source-independent acceptance, rejection, history, recomputation, process-model-evaluation, and active-input diagnostics. A factor result SHALL store one status and expose acceptance through `isAccepted()`. Latest-source diagnostics SHALL NOT move backward when an older delayed factor arrives after a newer factor from the same source.

#### Scenario: Invalid factor
- **WHEN** a factor contains invalid timing, dimensions, non-finite values, or unusable uncertainty
- **THEN** the factor is rejected atomically, the prior estimate remains unchanged, and diagnostics identify the rejection reason

#### Scenario: Delayed source timestamp
- **WHEN** an accepted delayed factor is older than the latest accepted factor from the same source
- **THEN** the latest-source diagnostic retains the newer timestamp

