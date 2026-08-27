# swift-ballistics

A Swift port of the [libballistics](https://github.com/grimwm/libballistics) library, designed for accurate and efficient ballistics simulation. This library provides tools to simulate projectile trajectories, accounting for various physical forces and environmental factors.

## Features

- **Accurate trajectory calculations** with 3-DOF numerical integration and continuous query interpolation
- **Aerodynamic Drag Models**: Support for standard **G1, G2, G5, G6, G7, and G8** profiles via `DragFunction`
- **Dynamic Speed of Sound**: Real-time thermodynamic speed of sound $c(T)$ based on atmospheric temperature
- **Point Blank Range (PBR)** solver for vital zone target optimization
- **Integrated Advanced Long-Range Physics**:
  - Gyroscopic Spin Drift (`SpinDrift`) with Miller stability factor ($S_g$)
  - Earth rotation deflections (`Coriolis` horizontal & Eötvös vertical effects)
  - Detailed point metrics (`totalWindage`, `totalDrop`, `totalWindageCorrection`, `totalDropCorrection`)
- **Bullet Property Utilities**: Sectional density ($SD$), form factor ($i$), and ballistic coefficient reconstruction via `SectionalDensity`
- **Atmospheric corrections** (altitude, barometric pressure, temperature, relative humidity)
- **Type-Safe Units**: Native integration with Foundation `Measurement` (`UnitLength`, `UnitSpeed`, `UnitAngle`, `UnitMass`, `UnitEnergy`, `UnitPressure`, `UnitTemperature`, `UnitDuration`)
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

### 1. Calculate Ballistic Trajectory (with Advanced Long-Range Options)

```swift
import Ballistics

// Generate a comprehensive ballistic solution
let solution = Ballistics.solve(
    preferredDistanceUnit: .yards, // Distance units used for trajectory sampling
    dragFunction: .g7, // .g1, .g2, .g5, .g6, .g7, .g8
    dragCoefficient: 0.265, // Projectile ballistic coefficient
    initialVelocity: Measurement(value: 2750, unit: .feetPerSecond), // Initial muzzle velocity
    sightHeight: Measurement(value: 1.5, unit: .inches), // Sight offset from bore
    shootingAngle: Measurement(value: 0, unit: .degrees), // Firing angle (+ up, - down)
    zeroRange: Measurement(value: 100, unit: .yards), // Zeroed distance
    atmosphere: Atmosphere(
        altitude: Measurement(value: 1500, unit: .feet),
        pressure: Measurement(value: 29.92, unit: .inchesOfMercury),
        temperature: Measurement(value: 59, unit: .fahrenheit),
        relativeHumidity: 0.5
    ),
    windSpeed: Measurement(value: 10, unit: .milesPerHour),
    windAngle: 90, // 0=headwind, 90=left-to-right, 180=tailwind, 270=right-to-left
    weight: Measurement(value: 175, unit: .grains),
    twist: Measurement(value: 10, unit: .inches), // 1:10" twist rate
    twistDirection: .right,
    bulletDiameter: Measurement(value: 0.308, unit: .inches),
    bulletLength: Measurement(value: 1.24, unit: .inches),
    latitude: Measurement(value: 45, unit: .degrees), // 45° N latitude
    azimuth: Measurement(value: 90, unit: .degrees) // Shooting East
)

// Smooth continuous interpolation supported at any arbitrary distance!
if let point = solution.getPoint(at: Measurement(value: 735.5, unit: .yards)) {
    print("Range: \(point.range)")
    print("Drop: \(point.drop)")
    print("Drop Correction: \(point.dropCorrection)")
    print("Crosswind Drift: \(point.windage)")
    print("Spin Drift: \(point.spinDrift ?? Measurement(value: 0, unit: .inches))")
    print("Coriolis Horizontal: \(point.coriolisHorizontal ?? Measurement(value: 0, unit: .inches))")
    print("Total Windage: \(point.totalWindage)")
    print("Total Windage Correction: \(point.totalWindageCorrection)")
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

### 3. Sectional Density & Form Factor

```swift
// Calculate bullet sectional density (SD) and form factor (i)
let sd = SectionalDensity.calculate(
    weight: Measurement(value: 175, unit: .grains),
    diameter: Measurement(value: 0.308, unit: .inches)
) // ~0.2635

let formFactorG7 = SectionalDensity.formFactor(
    sectionalDensity: sd,
    ballisticCoefficient: 0.265
) // ~0.994
```
