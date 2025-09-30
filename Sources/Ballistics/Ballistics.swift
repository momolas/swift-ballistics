//
//  Ballistics.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

public struct Ballistics {

    public var distances: [Point] = []
    
    internal init(distances: [Point]) {
        self.distances = distances
    }

    public static func solve(
        dragModel: DragModel,
        ballisticCoefficient: Double,
        initialVelocity: Measurement<UnitSpeed>,
        sightHeight: Measurement<UnitLength>,
        shootingAngle: Measurement<UnitAngle>,
        zeroRange: Measurement<UnitLength>,
        atmosphere: Atmosphere? = nil,
        windSpeed: Measurement<UnitSpeed>,
        windAngle: Double,
        weight: Measurement<UnitMass>,
        scopeClickValue: Double? = nil
    ) -> Ballistics {
        let simulation = Simulation(
            dragModel: dragModel,
            ballisticCoefficient: ballisticCoefficient,
            initialVelocity: initialVelocity.converted(to: .metersPerSecond).value,
            sightHeight: sightHeight.converted(to: .meters).value,
            shootingAngle: shootingAngle.converted(to: .radians).value,
            zeroRange: zeroRange.converted(to: .meters).value,
            atmosphere: atmosphere ?? Atmosphere(),
            windSpeed: windSpeed.converted(to: .metersPerSecond).value,
            windAngle: windAngle,
            weight: weight.converted(to: .kilograms).value,
            scopeClickValue: scopeClickValue
        )
        return simulation.run()
    }

    public func getPoint(at distance: Measurement<UnitLength>) -> Point? {
        let meters = distance.converted(to: .meters).value
        guard !distances.isEmpty else { return nil }

        // If the requested distance is outside our simulation range, return the closest boundary point.
        if meters <= distances.first!.range { return distances.first }
        if meters >= distances.last!.range { return distances.last }

        // Find the two points that bracket the target distance.
        guard let index = distances.firstIndex(where: { $0.range > meters }) else {
            return distances.last // Should not happen due to the checks above, but for safety.
        }

        let p1 = distances[index - 1]
        let p2 = distances[index]

        // If the distance is exactly at a point, no need to interpolate.
        if p2.range == meters {
            return p2
        }

        // Calculate the interpolation factor.
        let factor = (meters - p1.range) / (p2.range - p1.range)

        // Interpolate continuous values.
        let drop = p1.drop + factor * (p2.drop - p1.drop)
        let windage = p1.windage + factor * (p2.windage - p1.windage)
        let seconds = p1.seconds + factor * (p2.seconds - p1.seconds)
        let velocity = p1.velocity + factor * (p2.velocity - p1.velocity)
        let velocityX = p1.velocityX + factor * (p2.velocityX - p1.velocityX)
        let velocityY = p1.velocityY + factor * (p2.velocityY - p1.velocityY)
        let energy = p1.energy + factor * (p2.energy - p1.energy)

        // Create and return the new interpolated point.
        // Clicks are discrete and cannot be accurately interpolated, so they are omitted.
        return Point(
            range: meters,
            drop: drop,
            windage: windage,
            seconds: seconds,
            velocity: velocity,
            velocityX: velocityX,
            velocityY: velocityY,
            energy: energy,
            dropCorrectionClicks: nil,
            windageCorrectionClicks: nil
        )
    }
}

// Internal Simulation class to handle the actual calculation
internal struct Simulation {
    // input parameters - all in SI units
    let dragModel: DragModel
    let ballisticCoefficient: Double
    let initialVelocity: Double
    let sightHeight: Double
    let shootingAngle: Double
    let zeroRange: Double
    let atmosphere: Atmosphere
    let windSpeed: Double
    let windAngle: Double
    let weight: Double
    let scopeClickValue: Double?

    init(
        dragModel: DragModel,
        ballisticCoefficient: Double,
        initialVelocity: Double,
        sightHeight: Double,
        shootingAngle: Double,
        zeroRange: Double,
        atmosphere: Atmosphere,
        windSpeed: Double,
        windAngle: Double,
        weight: Double,
        scopeClickValue: Double?
    ) {
        self.dragModel = dragModel
        self.ballisticCoefficient = ballisticCoefficient
        self.initialVelocity = initialVelocity
        self.sightHeight = sightHeight
        self.shootingAngle = shootingAngle
        self.zeroRange = zeroRange
        self.atmosphere = atmosphere
        self.windSpeed = windSpeed
        self.windAngle = windAngle
        self.weight = weight
        self.scopeClickValue = scopeClickValue
    }

    // Main entry point to run the full simulation and get a trajectory table
    func run() -> Ballistics {
        let zeroAngle = Angle.zeroAngle(for: self)
        let distances = solveTrajectory(launchAngle: zeroAngle)
        return Ballistics(distances: distances)
    }

    // Solves for the drop at a specific range given a launch angle.
    // Used by the zeroAngle solver.
    internal func solveDropAtRange(launchAngle: Double, targetRange: Double) -> Double {
        var vx = initialVelocity * cos(launchAngle)
        var vy = initialVelocity * sin(launchAngle)
        var x: Double = 0
        var y: Double = -sightHeight

        let headwind = windSpeed * cos(Math.degToRad(windAngle))
        let gy = Constants.gravity * cos(shootingAngle + launchAngle)
        let gx = Constants.gravity * sin(shootingAngle + launchAngle)
        let airDensity = atmosphere.airDensity()

        while x < targetRange {
            let (dt, newVx, newVy) = step(vx: vx, vy: vy, headwind: headwind, gx: gx, gy: gy, airDensity: airDensity)
            guard dt > 0 else { break }

            vx = newVx
            vy = newVy

            x += dt * vx
            y += dt * vy

            if abs(vy) > abs(Constants.simulationStopSlope * vx) {
                break
            }
        }
        return y
    }

    // Solves the full trajectory and returns an array of Points
    private func solveTrajectory(launchAngle: Double) -> [Point] {
        var points: [Point] = []
        var vx = initialVelocity * cos(launchAngle)
        var vy = initialVelocity * sin(launchAngle)
        var x: Double = 0
        var y: Double = -sightHeight
        var t: Double = 0
        var windage: Double = 0
        var n = 0

        let headwind = windSpeed * cos(Math.degToRad(windAngle))
        let crosswind = windSpeed * sin(Math.degToRad(windAngle))
        let gy = Constants.gravity * cos(shootingAngle + launchAngle)
        let gx = Constants.gravity * sin(shootingAngle + launchAngle)
        let airDensity = atmosphere.airDensity()

        while true {
            let v = sqrt(vx * vx + vy * vy)
            if v == 0 { break }

            if x >= Double(n) {
                let energy = 0.5 * weight * v * v

                var dropClicks: Int? = nil
                var windageClicks: Int? = nil

                if let clickValue = self.scopeClickValue, clickValue > 0 {
                    let dropMoa = -Math.radToMOA(atan(y / x))
                    let windageMoa = Math.radToMOA(atan(windage / x))
                    dropClicks = Int(round(dropMoa / clickValue))
                    windageClicks = Int(round(windageMoa / clickValue))
                }

                let point = Point(
                    range: x,
                    drop: y,
                    windage: windage,
                    seconds: t,
                    velocity: v,
                    velocityX: vx,
                    velocityY: vy,
                    energy: energy,
                    dropCorrectionClicks: dropClicks,
                    windageCorrectionClicks: windageClicks
                )
                points.append(point)

                n += 1
            }

            let (dt, newVx, newVy) = step(vx: vx, vy: vy, headwind: headwind, gx: gx, gy: gy, airDensity: airDensity)
            guard dt > 0 else { break }

            vx = newVx
            vy = newVy

            x += dt * vx
            y += dt * vy
            windage += crosswind * dt
            t += dt

            if abs(vy) > abs(Constants.simulationStopSlope * vx) || x >= Constants.maxRange {
                break
            }
        }
        return points
    }

    // A single step in the simulation, returns the time delta and new velocities
    private func step(vx: Double, vy: Double, headwind: Double, gx: Double, gy: Double, airDensity: Double) -> (dt: Double, newVx: Double, newVy: Double) {
        let v = sqrt(vx * vx + vy * vy)
        guard v > 0 else { return (0, vx, vy) }

        let v_air = v + headwind
        let cd = Drag.coefficient(for: self.dragModel, velocity: v_air, atmosphere: self.atmosphere)
        let retardation = (0.5 * airDensity * v_air * v_air * cd) / self.ballisticCoefficient

        let dvx = -(vx / v) * retardation
        let dvy = -(vy / v) * retardation

        let dt = Constants.timeStepFactor / v

        let newVx = vx + dt * dvx + dt * gx
        let newVy = vy + dt * dvy + dt * gy

        return (dt, newVx, newVy)
    }
}
