//
//  Ballistics.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

public struct Ballistics {

    // Stores sampled points along the trajectory at fixed distance steps in the preferred unit.
    var distances: [Point] = []

    // Sampling configuration captured so getPoint(at:) can map requests to indices correctly.
    private let preferredDistanceUnit: UnitLength
    private let distanceStep: Measurement<UnitLength>

    init(
        preferredDistanceUnit: UnitLength = .yards,
        distanceStep: Measurement<UnitLength> = Measurement(value: 1, unit: .yards)
    ) {
        self.preferredDistanceUnit = preferredDistanceUnit
        self.distanceStep = distanceStep.converted(to: preferredDistanceUnit)
    }

    /**
     Solves the projectile trajectory based on provided ballistic and environmental parameters.

     This method calculates the trajectory of a projectile, considering factors like aerodynamic drag,
     initial velocity, sight height, shooting angle, zeroing angle, and wind conditions.
     It returns key trajectory points, allowing for analysis of the projectile's flight path.

     - Parameters:
       - preferredDistanceUnit: The preferred distance unit, used for sampling
       - dragCoefficient: The G1 drag coefficient of the projectile, a dimensionless number representing aerodynamic resistance.
       - initialVelocity: The muzzle velocity of the projectile in meters per second (ft/s).
       - sightHeight: The height of the sight above the bore axis in inches (in).
       - shootingAngle: The actual angle of elevation at which the projectile is fired, in degrees. Positive is up, negative is down.
       - zeroRange: The distance the projectile is zeroed at.
       - atmosphere: The atmospheric conditions to consider. Optional.
       - windSpeed: The speed of the wind.
       - windAngle: The direction of the wind relative to the projectile's path, in degrees (0° = headwind, 90° = left to right).
       - weight: The projectile weight.
       - preferredDistanceUnit: The unit in which you want sampling to occur (e.g., .yards or .meters). Default is .yards.
       - distanceStep: The sampling step in the preferred unit. Default is 1 yard.

     - Returns:
       A ballistics object, which contains a set of trajectory points sampled at fixed distance steps
       in the preferred unit.
    */
    public static func solve(
        preferredDistanceUnit: UnitLength = .yards,
        dragCoefficient: Double,
        initialVelocity: Measurement<UnitSpeed>,
        sightHeight: Measurement<UnitLength>,
        shootingAngle: Measurement<UnitAngle>,
        zeroRange: Measurement<UnitLength>,
        atmosphere: Atmosphere? = nil,
        windSpeed: Measurement<UnitSpeed>,
        windAngle: Double,
        weight: Measurement<UnitMass> = Measurement<UnitMass>(value: 0, unit: .grains),
        distanceStep: Measurement<UnitLength> = Measurement(value: 1, unit: .yards)
    ) -> Ballistics {
        
        var ballistics = Ballistics(
            preferredDistanceUnit: preferredDistanceUnit,
            distanceStep: distanceStep
        )

        // Normalize step to feet for threshold checks; keep a clean preferred-unit step for labeling.
        let stepInPreferred = distanceStep.converted(to: preferredDistanceUnit)
        let stepFeet = stepInPreferred.converted(to: .feet).value
        let unitSymbol = preferredDistanceUnit.symbol

        // Compute a maximum range in feet based on existing yard-based constant for compatibility.
        // If preferred unit is meters, convert the same max-yards distance to feet.
        let maxFeetFromYards = Double(Constants.BALLISTICS_COMPUTATION_MAX_YARDS) * 3.0
        let maxFeet = maxFeetFromYards

        let environmentDragCoefficient = atmosphere?.adjustCoefficient(dragCoefficient: dragCoefficient) ?? dragCoefficient
        let initialVelocityFPS = initialVelocity.converted(to: .feetPerSecond).value
        let zeroAngle = Angle.zeroAngle(
            dragCoefficient: environmentDragCoefficient,
            initialVelocity: initialVelocity,
            sightHeight: sightHeight,
            zeroRange: zeroRange,
            yIntercept: 0
        )
        let headwind = headwindSpeed(windSpeed: windSpeed.converted(to: .milesPerHour).value, windAngle: windAngle)
        let crosswind = crosswindSpeed(windSpeed: windSpeed.converted(to: .milesPerHour).value, windAngle: windAngle)
        let gy = Constants.GRAVITY * cos(Math.degToRad(shootingAngle.converted(to: .degrees).value + zeroAngle))
        let gx = Constants.GRAVITY * sin(Math.degToRad(shootingAngle.converted(to: .degrees).value + zeroAngle))

        var vx = initialVelocityFPS * cos(Math.degToRad(zeroAngle))
        var vy = initialVelocityFPS * sin(Math.degToRad(zeroAngle))
        var x: Double = 0 // feet
        var y: Double = -sightHeight.converted(to: .inches).value / 12 // feet (inches / 12)
        var t: Double = 0

        // Sampling index and next threshold in feet
        var sampleIndex = 0
        var nextSampleFeet = Double(sampleIndex) * stepFeet

        // Emit initial point at 0 distance
        func emitPoint(currentV: Double, elapsed: Double, vx: Double, vy: Double, xFeet: Double, yFeet: Double) {
            let pathInches = yFeet * 12
            let moaDrop = -Math.radToMOA(atan(yFeet / max(xFeet, 1e-9)))
            let windageInches = Ballistics.windage(windSpeed: crosswind, initialVelocity: initialVelocityFPS, x: xFeet, t: elapsed)
            let moaWindage = Math.radToMOA(atan((windageInches / 12) / max(xFeet, 1e-9)))
            let ftlbs = weight.converted(to: .grains).value * (pow(currentV, 2)) / (2 * 32.163 * 7000)

            // Range labeling in preferred unit aligned to exact sample index
            let rangeValue = Double(sampleIndex) * stepInPreferred.value
            let point = Point(
                range: Measurement(value: rangeValue, unit: ballistics.preferredDistanceUnit),
                drop: Measurement(value: pathInches, unit: .inches),
                dropCorrection: Measurement(value: moaDrop, unit: .minutesOfAngle),
                windage: Measurement(value: windageInches, unit: .inches),
                windageCorrection: Measurement(value: moaWindage, unit: .minutesOfAngle),
                travelTime: Measurement(value: elapsed, unit: .seconds),
                velocity: Measurement(value: currentV, unit: .feetPerSecond),
                velocityX: Measurement(value: vx, unit: .feetPerSecond),
                velocityY: Measurement(value: vy, unit: .feetPerSecond),
                energy: Measurement(value: ftlbs, unit: .footPounds)
            )
            ballistics.distances.append(point)
        }

        // Ensure first sample at 0 distance
        emitPoint(currentV: sqrt(vx * vx + vy * vy), elapsed: t, vx: vx, vy: vy, xFeet: x, yFeet: y)
        sampleIndex += 1
        nextSampleFeet = Double(sampleIndex) * stepFeet

        while true {
            let v = sqrt(vx * vx + vy * vy)
            let dv = Drag.retard(dragCoefficient: environmentDragCoefficient, projectileVelocity: v + headwind)
            let dvx = -(vx / max(v, 1e-9)) * dv
            let dvy = -(vy / max(v, 1e-9)) * dv

            let dt = 0.5 / max(v, 1e-9)
            let vxNext = vx + dt * dvx + dt * gx
            let vyNext = vy + dt * dvy + dt * gy

            let xNext = x + dt * (vx + vxNext) / 2
            let yNext = y + dt * (vy + vyNext) / 2

            // Check if we crossed the next sample threshold (in feet)
            while xNext >= nextSampleFeet {
                // Linear interpolation to the exact sample position for reporting
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

            // Advance state
            x = xNext
            y = yNext
            vx = vxNext
            vy = vyNext
            t += dt

            // Termination conditions
            if abs(vy) > abs(3 * vx) { break }              // trajectory too steep
            if x >= maxFeet { break }                        // exceeded max range
        }

        return ballistics
    }

    public func getPoint(at distance: Measurement<UnitLength>) -> Point? {
        // Convert requested distance to preferred unit and compute the sample index
        let requestedInPreferred = distance.converted(to: preferredDistanceUnit).value
        let stepValue = distanceStep.value
        guard stepValue > 0 else { return nil }

        let index = Int((requestedInPreferred / stepValue).rounded())
        guard index >= 0 && index < distances.count else { return nil }
        return distances[index]
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
}
