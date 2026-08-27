# swift-ballistics

A Swift port of the [libballistics](https://github.com/grimwm/libballistics) library, designed for accurate and efficient ballistics simulation. This library provides tools to simulate projectile trajectories, accounting for various physical forces and environmental factors.

## Features

- **Accurate trajectory calculations** with numerical integration and customizable sampling
- **Aerodynamic Drag Models**: Support for **G1** (standard sporting bullets) and **G7** (long-range boat-tail/VLD bullets) via `DragFunction`
- **Point Blank Range (PBR)** solver for vital zone target optimization
- **Advanced Long-Range Physics**:
  - Gyroscopic Spin Drift (`SpinDrift`) with Miller stability factor ($S_g$)
  - Earth rotation deflections (`Coriolis` & Eötvös effect)
- **Atmospheric corrections** (altitude, barometric pressure, temperature, relative humidity)
- **Type-Safe Units**: Native integration with Foundation `Measurement` (`UnitLength`, `UnitSpeed`, `UnitAngle`, `UnitMass`, `UnitEnergy`, `UnitPressure`, `UnitTemperature`)
- Concurrency-ready (`Sendable`, Swift 6 strict mode)

## Installation

### Swift Package Manager (SPM)

To include `swift-ballistics` in your project, add it as a dependency in your `Package.swift` file:

```swift
dependencies: [
    .package(url: "https://github.com/raydowe/swift-ballistics.git", .upToNextMajor(from: "3.0.0"))
]
```

## Usage

### 1. Calculate Ballistic Trajectory

```swift
import Ballistics

// Generate a full ballistic solution (supporting .g1 or .g7)
let solution = Ballistics.solve(
    preferredDistanceUnit: .yards, // The preferred distance units used for sampling
    dragFunction: .g1, // .g1 or .g7
    dragCoefficient: 0.414, // The drag coefficient of the projectile
    initialVelocity: Measurement(value: 3300, unit: .feetPerSecond), // Initial muzzle velocity
    sightHeight: Measurement(value: 1.8, unit: .inches), // Sight offset from bore
    shootingAngle: Measurement(value: 0, unit: .degrees), // Firing angle (+ up, - down)
    zeroRange: Measurement(value: 100, unit: .yards), // Zeroed distance
    atmosphere: Atmosphere(
        altitude: Measurement(value: 10_000, unit: .feet),
        pressure: Measurement(value: 30.1, unit: .inchesOfMercury),
        temperature: Measurement(value: 5, unit: .fahrenheit),
        relativeHumidity: 0.5
    ),
    windSpeed: Measurement(value: 20, unit: .milesPerHour),
    windAngle: 135, // 0=headwind, 90=left-to-right, 180=tailwind, 270=right-to-left
    weight: Measurement(value: 120, unit: .grains)
)

// Read point values at a given range
if let point = solution.getPoint(at: Measurement(value: 200, unit: .yards)) {
    print("Exact range: \(point.range)")
    print("Drop: \(point.drop)")
    print("Drop Correction: \(point.dropCorrection)")
    print("Windage: \(point.windage)")
    print("Windage Correction: \(point.windageCorrection)")
    print("Velocity: \(point.velocity)")
    print("Energy: \(point.energy)")
    print("Travel Time: \(point.travelTime)")
}
```

### 2. Point Blank Range (PBR)

```swift
// Solve for an 8-inch vital zone target
let pbrResult = PBR.solve(
    dragFunction: .g1,
    dragCoefficient: 0.414,
    initialVelocity: Measurement(value: 3000, unit: .feetPerSecond),
    sightHeight: Measurement(value: 1.5, unit: .inches),
    vitalSize: Measurement(value: 8, unit: .inches)
)

if case .success(let pbr) = pbrResult {
    print("Near Zero: \(pbr.nearZeroYards) yards")
    print("Far Zero: \(pbr.farZeroYards) yards")
    print("Max PBR: \(pbr.maxPBRYards) yards")
    print("Sight-in offset @ 100 yds: \(Double(pbr.sightInAt100Yards) / 100.0) inches")
}
```

### 3. Spin Drift & Coriolis Effect

```swift
// Gyroscopic stability (Sg) & spin drift
let sg = SpinDrift.stabilityFactor(
    weight: Measurement(value: 175, unit: .grains),
    diameter: Measurement(value: 0.308, unit: .inches),
    length: Measurement(value: 1.24, unit: .inches),
    twist: Measurement(value: 10, unit: .inches) // 1:10" twist
)

let drift = SpinDrift.deflection(
    timeOfFlight: Measurement(value: 1.0, unit: .seconds),
    stabilityFactor: sg,
    twistDirection: .right
)

// Coriolis effect (Earth rotation)
let coriolis = Coriolis.deflection(
    latitude: Measurement(value: 45, unit: .degrees),
    azimuth: Measurement(value: 90, unit: .degrees), // Shooting East
    range: Measurement(value: 1000, unit: .yards),
    timeOfFlight: Measurement(value: 1.5, unit: .seconds)
)
print("Coriolis Horizontal: \(coriolis.horizontal)")
print("Coriolis Vertical (Eötvös): \(coriolis.vertical)")
```
