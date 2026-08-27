//
//  Ballistics.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

public struct Ballistics: Sendable, Equatable, Hashable {

    /// Stores sampled points along the trajectory at fixed distance steps in the preferred unit.
    public internal(set) var distances: [Point] = []

    /// The preferred distance unit used for trajectory sampling.
    public let preferredDistanceUnit: UnitLength

    /// The sampling step distance in preferred units.
    public let distanceStep: Measurement<UnitLength>

    public init(
        preferredDistanceUnit: UnitLength = .yards,
        distanceStep: Measurement<UnitLength> = Measurement(value: 1, unit: .yards)
    ) {
        self.preferredDistanceUnit = preferredDistanceUnit
        self.distanceStep = distanceStep.converted(to: preferredDistanceUnit)
    }

    /**
     Solves the projectile trajectory based on provided ballistic and environmental parameters.

     This method calculates the trajectory of a projectile, considering aerodynamic drag (G1–G8),
     initial velocity, sight height, shooting angle, zeroing angle, wind conditions, and optionally
     rifling twist (gyroscopic spin drift) and Earth rotation (Coriolis / Eötvös deflections).

     - Parameters:
       - preferredDistanceUnit: The preferred distance unit, used for sampling (e.g., .yards or .meters). Default is .yards.
       - dragFunction: The aerodynamic drag function (.g1, .g2, .g5, .g6, .g7, .g8). Default is .g1.
       - dragCoefficient: The drag coefficient of the projectile.
       - initialVelocity: The muzzle velocity of the projectile.
       - sightHeight: The height of the sight above the bore axis.
       - shootingAngle: The actual angle of elevation at which the projectile is fired. Positive is up, negative is down.
       - zeroRange: The distance the projectile is zeroed at.
       - atmosphere: The atmospheric conditions to consider (temperature, pressure, altitude, humidity). Optional.
       - windSpeed: The speed of the wind.
       - windAngle: The direction of the wind relative to the projectile's path, in degrees (0° = headwind, 90° = left to right).
       - weight: The projectile weight.
       - distanceStep: The sampling step in the preferred unit. Default is 1 yard.
       - twist: Barrel rifling twist rate (e.g. 1 turn in 10 inches). Optional.
       - twistDirection: Barrel rifling twist direction (.right or .left). Default is .right.
       - bulletDiameter: Projectile caliber/diameter. Optional, used for Miller stability factor.
       - bulletLength: Projectile length. Optional, used for Miller stability factor.
       - latitude: Firing position latitude. Optional, used for Coriolis deflections.
       - azimuth: Shooting compass azimuth. Optional, used for Coriolis deflections.

     - Returns:
       A ballistics object containing trajectory points sampled at regular distance steps with continuous query capability.
    */
    public static func solve(
        preferredDistanceUnit: UnitLength = .yards,
        dragFunction: DragFunction = .g1,
        dragCoefficient: Double,
        initialVelocity: Measurement<UnitSpeed>,
        sightHeight: Measurement<UnitLength>,
        shootingAngle: Measurement<UnitAngle>,
        zeroRange: Measurement<UnitLength>,
        atmosphere: Atmosphere? = nil,
        windSpeed: Measurement<UnitSpeed>,
        windAngle: Double,
        weight: Measurement<UnitMass> = Measurement<UnitMass>(value: 0, unit: .grains),
        distanceStep: Measurement<UnitLength> = Measurement(value: 1, unit: .yards),
        twist: Measurement<UnitLength>? = nil,
        twistDirection: TwistDirection = .right,
        bulletDiameter: Measurement<UnitLength>? = nil,
        bulletLength: Measurement<UnitLength>? = nil,
        latitude: Measurement<UnitAngle>? = nil,
        azimuth: Measurement<UnitAngle>? = nil
    ) -> Ballistics {
        
        var ballistics = Ballistics(
            preferredDistanceUnit: preferredDistanceUnit,
            distanceStep: distanceStep
        )

        let stepInPreferred = distanceStep.converted(to: preferredDistanceUnit)
        let stepFeet = stepInPreferred.converted(to: .feet).value
        let maxFeet = Double(Constants.BALLISTICS_COMPUTATION_MAX_YARDS) * 3.0

        let environmentDragCoefficient = atmosphere?.adjustCoefficient(dragCoefficient: dragCoefficient) ?? dragCoefficient
        let soundSpeedFPS = atmosphere?.speedOfSound.converted(to: .feetPerSecond).value ?? Drag.defaultSpeedOfSoundFPS

        let initialVelocityFPS = initialVelocity.converted(to: .feetPerSecond).value
        let zeroAngle = Angle.zeroAngle(
            dragFunction: dragFunction,
            dragCoefficient: environmentDragCoefficient,
            initialVelocity: initialVelocity,
            sightHeight: sightHeight,
            zeroRange: zeroRange,
            yIntercept: 0,
            speedOfSoundFPS: soundSpeedFPS
        )

        let headwind = headwindSpeed(windSpeed: windSpeed.converted(to: .milesPerHour).value, windAngle: windAngle)
        let crosswind = crosswindSpeed(windSpeed: windSpeed.converted(to: .milesPerHour).value, windAngle: windAngle)
        let gy = Constants.GRAVITY * cos(Math.degToRad(shootingAngle.converted(to: .degrees).value + zeroAngle))
        let gx = Constants.GRAVITY * sin(Math.degToRad(shootingAngle.converted(to: .degrees).value + zeroAngle))

        // Precompute stability factor if bullet dimensions and twist are provided
        let stabilityFactor: Double? = {
            guard let t = twist else { return nil }
            if let d = bulletDiameter, let l = bulletLength, weight.value > 0 {
                return SpinDrift.stabilityFactor(
                    weight: weight,
                    diameter: d,
                    length: l,
                    twist: t,
                    muzzleVelocity: initialVelocity
                )
            }
            return 1.5
        }()

        var vx = initialVelocityFPS * cos(Math.degToRad(zeroAngle))
        var vy = initialVelocityFPS * sin(Math.degToRad(zeroAngle))
        var x: Double = 0 // feet
        var y: Double = -sightHeight.converted(to: .inches).value / 12 // feet
        var t: Double = 0

        var sampleIndex = 0
        var nextSampleFeet = Double(sampleIndex) * stepFeet

        func emitPoint(currentV: Double, elapsed: Double, vx: Double, vy: Double, xFeet: Double, yFeet: Double) {
            let pathInches = yFeet * 12.0
            let moaDrop = -Math.radToMOA(atan(yFeet / max(xFeet, 1e-9)))
            let windageInches = Ballistics.windage(windSpeed: crosswind, initialVelocity: initialVelocityFPS, x: xFeet, t: elapsed)
            let moaWindage = Math.radToMOA(atan((windageInches / 12.0) / max(xFeet, 1e-9)))
            let ftlbs = weight.converted(to: .grains).value * (pow(currentV, 2)) / (2 * 32.163 * 7000)

            let duration = Measurement(value: elapsed, unit: UnitDuration.seconds)
            let rangeMeasurement = Measurement(value: Double(sampleIndex) * stepInPreferred.value, unit: ballistics.preferredDistanceUnit)

            // Calculate optional spin drift
            let spinDriftMeasurement: Measurement<UnitLength>? = {
                guard let sg = stabilityFactor else { return nil }
                return SpinDrift.deflection(timeOfFlight: duration, stabilityFactor: sg, twistDirection: twistDirection)
            }()

            // Calculate optional Coriolis deflections
            let coriolisResult: (horizontal: Measurement<UnitLength>, vertical: Measurement<UnitLength>)? = {
                guard let lat = latitude, let az = azimuth else { return nil }
                return Coriolis.deflection(latitude: lat, azimuth: az, range: rangeMeasurement, timeOfFlight: duration)
            }()

            let point = Point(
                range: rangeMeasurement,
                drop: Measurement(value: pathInches, unit: .inches),
                dropCorrection: Measurement(value: moaDrop, unit: .minutesOfAngle),
                windage: Measurement(value: windageInches, unit: .inches),
                windageCorrection: Measurement(value: moaWindage, unit: .minutesOfAngle),
                travelTime: duration,
                velocity: Measurement(value: currentV, unit: .feetPerSecond),
                velocityX: Measurement(value: vx, unit: .feetPerSecond),
                velocityY: Measurement(value: vy, unit: .feetPerSecond),
                energy: Measurement(value: ftlbs, unit: .footPounds),
                spinDrift: spinDriftMeasurement,
                coriolisHorizontal: coriolisResult?.horizontal,
                coriolisVertical: coriolisResult?.vertical
            )
            ballistics.distances.append(point)
        }

        emitPoint(currentV: sqrt(vx * vx + vy * vy), elapsed: t, vx: vx, vy: vy, xFeet: x, yFeet: y)
        sampleIndex += 1
        nextSampleFeet = Double(sampleIndex) * stepFeet

        while true {
            let v = sqrt(vx * vx + vy * vy)
            let dv = Drag.retard(
                dragFunction: dragFunction,
                dragCoefficient: environmentDragCoefficient,
                projectileVelocity: v + headwind,
                speedOfSoundFPS: soundSpeedFPS
            )
            let dvx = -(vx / max(v, 1e-9)) * dv
            let dvy = -(vy / max(v, 1e-9)) * dv

            let dt = 0.5 / max(v, 1e-9)
            let vxNext = vx + dt * dvx + dt * gx
            let vyNext = vy + dt * dvy + dt * gy

            let xNext = x + dt * (vx + vxNext) / 2.0
            let yNext = y + dt * (vy + vyNext) / 2.0

            while xNext >= nextSampleFeet {
                let alpha = (nextSampleFeet - x) / max(xNext - x, 1e-12)
                let vxInterp = vx + alpha * (vxNext - vx)
                let vyInterp = vy + alpha * (vyNext - vy)
                let vInterp = sqrt(vxInterp * vxInterp + vyInterp * vyInterp)
                let yInterp = y + alpha * (yNext - y)
                let tInterp = t + alpha * dt

                emitPoint(currentV: vInterp, elapsed: tInterp, vx: vxInterp, vy: vyInterp, xFeet: nextSampleFeet, yFeet: yInterp)

                sampleIndex += 1
                nextSampleFeet = Double(sampleIndex) * stepFeet

                if nextSampleFeet > maxFeet { break }
            }

            x = xNext
            y = yNext
            vx = vxNext
            vy = vyNext
            t += dt

            if abs(vy) > abs(3 * vx) { break }
            if x >= maxFeet { break }
        }

        return ballistics
    }

    /**
     Retrieves the ballistic point at the specified target distance.
     Performs smooth continuous linear interpolation if the requested distance falls between two sampled distance steps.

     - Parameter distance: The distance at which to retrieve or interpolate the ballistic metrics.
     - Returns: The interpolated `Point`, or `nil` if out of bounds.
     */
    public func getPoint(at distance: Measurement<UnitLength>) -> Point? {
        guard !distances.isEmpty else { return nil }

        let requestedInPreferred = distance.converted(to: preferredDistanceUnit).value
        let stepValue = distanceStep.value
        guard stepValue > 0 else { return nil }

        let exactIndex = requestedInPreferred / stepValue
        guard exactIndex >= 0 else { return nil }

        let lowerIndex = Int(floor(exactIndex))
        let upperIndex = Int(ceil(exactIndex))

        guard lowerIndex < distances.count else { return nil }

        if lowerIndex == upperIndex || upperIndex >= distances.count {
            return distances[lowerIndex]
        }

        let p0 = distances[lowerIndex]
        let p1 = distances[upperIndex]
        let factor = exactIndex - Double(lowerIndex)

        let interpDrop = p0.drop.value + factor * (p1.drop.value - p0.drop.value)
        let interpDropCorr = p0.dropCorrection.value + factor * (p1.dropCorrection.value - p0.dropCorrection.value)
        let interpWindage = p0.windage.value + factor * (p1.windage.value - p0.windage.value)
        let interpWindageCorr = p0.windageCorrection.value + factor * (p1.windageCorrection.value - p0.windageCorrection.value)
        let interpTime = p0.travelTime.value + factor * (p1.travelTime.value - p0.travelTime.value)
        let interpV = p0.velocity.value + factor * (p1.velocity.value - p0.velocity.value)
        let interpVx = p0.velocityX.value + factor * (p1.velocityX.value - p0.velocityX.value)
        let interpVy = p0.velocityY.value + factor * (p1.velocityY.value - p0.velocityY.value)
        let interpEnergy = p0.energy.value + factor * (p1.energy.value - p0.energy.value)

        let interpSpinDrift: Measurement<UnitLength>? = {
            guard let s0 = p0.spinDrift?.value, let s1 = p1.spinDrift?.value else { return nil }
            return Measurement(value: s0 + factor * (s1 - s0), unit: .inches)
        }()

        let interpCoriolisHoriz: Measurement<UnitLength>? = {
            guard let c0 = p0.coriolisHorizontal?.value, let c1 = p1.coriolisHorizontal?.value else { return nil }
            return Measurement(value: c0 + factor * (c1 - c0), unit: .inches)
        }()

        let interpCoriolisVert: Measurement<UnitLength>? = {
            guard let c0 = p0.coriolisVertical?.value, let c1 = p1.coriolisVertical?.value else { return nil }
            return Measurement(value: c0 + factor * (c1 - c0), unit: .inches)
        }()

        return Point(
            range: distance,
            drop: Measurement(value: interpDrop, unit: p0.drop.unit),
            dropCorrection: Measurement(value: interpDropCorr, unit: p0.dropCorrection.unit),
            windage: Measurement(value: interpWindage, unit: p0.windage.unit),
            windageCorrection: Measurement(value: interpWindageCorr, unit: p0.windageCorrection.unit),
            travelTime: Measurement(value: interpTime, unit: p0.travelTime.unit),
            velocity: Measurement(value: interpV, unit: p0.velocity.unit),
            velocityX: Measurement(value: interpVx, unit: p0.velocityX.unit),
            velocityY: Measurement(value: interpVy, unit: p0.velocityY.unit),
            energy: Measurement(value: interpEnergy, unit: p0.energy.unit),
            spinDrift: interpSpinDrift,
            coriolisHorizontal: interpCoriolisHoriz,
            coriolisVertical: interpCoriolisVert
        )
    }

    private static func headwindSpeed(windSpeed: Double, windAngle: Double) -> Double {
        return windSpeed * cos(Math.degToRad(windAngle))
    }

    private static func crosswindSpeed(windSpeed: Double, windAngle: Double) -> Double {
        return windSpeed * sin(Math.degToRad(windAngle))
    }

    private static func windage(windSpeed: Double, initialVelocity: Double, x: Double, t: Double) -> Double {
        let vw = windSpeed * 17.60 // Convert to inches per second
        return vw * (t - x / max(initialVelocity, 1e-9))
    }

    /**
     Solves a high-fidelity 6-DOF rigid-body trajectory (Lapua Ballistics standard) using 4th-order Runge-Kutta integration.
     */
    public static func solve6DOF(
        properties: ProjectileProperties,
        coefficients: AerodynamicCoefficients? = nil,
        dragFunction: DragFunction = .g7,
        dragCoefficient: Double = 0.500,
        initialVelocity: Measurement<UnitSpeed>,
        sightHeight: Measurement<UnitLength>,
        zeroRange: Measurement<UnitLength>,
        shootingAngle: Measurement<UnitAngle> = Measurement(value: 0, unit: .degrees),
        twist: Measurement<UnitLength>,
        twistDirection: TwistDirection = .right,
        atmosphere: Atmosphere? = nil,
        windSpeed: Measurement<UnitSpeed> = Measurement(value: 0, unit: .milesPerHour),
        windAngle: Double = 0,
        latitude: Measurement<UnitAngle>? = nil,
        azimuth: Measurement<UnitAngle>? = nil,
        distanceStep: Measurement<UnitLength> = Measurement(value: 1, unit: .yards),
        preferredDistanceUnit: UnitLength = .yards
    ) -> Ballistics {
        let aeroCoeffs = coefficients ?? AerodynamicCoefficients.synthesize(
            properties: properties,
            dragFunction: dragFunction,
            dragCoefficient: dragCoefficient
        )

        return Solver6DOF.solve(
            properties: properties,
            coefficients: aeroCoeffs,
            initialVelocity: initialVelocity,
            sightHeight: sightHeight,
            zeroRange: zeroRange,
            shootingAngle: shootingAngle,
            twist: twist,
            twistDirection: twistDirection,
            atmosphere: atmosphere,
            windSpeed: windSpeed,
            windAngle: windAngle,
            latitude: latitude,
            azimuth: azimuth,
            distanceStep: distanceStep,
            preferredDistanceUnit: preferredDistanceUnit
        )
    }
}
