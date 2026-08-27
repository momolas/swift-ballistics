//
//  Solver6DOF.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

/// High-fidelity 6-DOF (6 Degrees of Freedom) rigid-body trajectory solver using 4th-order Runge-Kutta (RK4) integration.
public struct Solver6DOF: Sendable {

    public struct Derivatives: Sendable {
        public var dx: Double
        public var dy: Double
        public var dz: Double
        public var dvx: Double
        public var dvy: Double
        public var dvz: Double
        public var dpitch: Double
        public var dyaw: Double
        public var droll: Double
        public var dp: Double
        public var dq: Double
        public var dr: Double
    }

    /**
     Solves the full 6-DOF rigid-body trajectory integrating 12 coupled differential equations.

     - Parameters:
       - properties: Projectile physical rigid-body properties (mass, dimensions, moments of inertia).
       - coefficients: 6-DOF aerodynamic force and moment derivatives.
       - initialVelocity: Muzzle velocity.
       - sightHeight: Sight offset above bore axis.
       - zeroRange: Firearm zero distance.
       - shootingAngle: Elevation angle of the shot (+ up, - down).
       - twist: Barrel rifling twist (e.g. 1:10 inches).
       - twistDirection: Barrel twist direction (.right or .left).
       - atmosphere: Ambient atmospheric conditions.
       - windSpeed: Wind speed.
       - windAngle: Wind angle (0° = headwind, 90° = left-to-right).
       - latitude: Firing latitude (for Coriolis).
       - azimuth: Shooting azimuth (for Coriolis).
       - distanceStep: Sampling step distance.
       - preferredDistanceUnit: Preferred unit for trajectory output.

     - Returns:
       A `Ballistics` solution containing rich 6-DOF metrics.
     */
    public static func solve(
        properties: ProjectileProperties,
        coefficients: AerodynamicCoefficients,
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

        var ballistics = Ballistics(
            preferredDistanceUnit: preferredDistanceUnit,
            distanceStep: distanceStep
        )

        let stepInPreferred = distanceStep.converted(to: preferredDistanceUnit)
        let stepFeet = stepInPreferred.converted(to: .feet).value
        let maxFeet = Double(Constants.BALLISTICS_COMPUTATION_MAX_YARDS) * 3.0

        let v0FPS = initialVelocity.converted(to: .feetPerSecond).value
        let twistInches = twist.converted(to: .inches).value
        let sightInches = sightHeight.converted(to: .inches).value
        let initialYFeet = -sightInches / 12.0

        let soundSpeedFPS = atmosphere?.speedOfSound.converted(to: .feetPerSecond).value ?? Drag.defaultSpeedOfSoundFPS
        let airDensitySlugFt3 = 0.0023769 // Standard sea level air density

        // Initial spin rate p0 = (2 * pi * V0) / (twist_in_feet) * twist_sign
        let twistFeet = max(0.1, twistInches / 12.0)
        let initialP = (2.0 * Double.pi * v0FPS / twistFeet) * twistDirection.sign

        // Initial zero angle estimation
        let zeroAngleDeg = Angle.zeroAngle(
            dragFunction: .g7,
            dragCoefficient: 0.500,
            initialVelocity: initialVelocity,
            sightHeight: sightHeight,
            zeroRange: zeroRange,
            yIntercept: 0,
            speedOfSoundFPS: soundSpeedFPS
        )

        let totalElevationAngleRad = Math.degToRad(shootingAngle.converted(to: .degrees).value + zeroAngleDeg)

        // Initial 6-DOF State
        var state = State6DOF(
            x: 0,
            y: initialYFeet,
            z: 0,
            vx: v0FPS * cos(totalElevationAngleRad),
            vy: v0FPS * sin(totalElevationAngleRad),
            vz: 0,
            pitch: totalElevationAngleRad,
            yaw: 0,
            roll: 0,
            p: initialP,
            q: 0,
            r: 0
        )

        let mass = properties.massSlugs
        let diamFeet = (properties.diameter.converted(to: .inches).value) / 12.0
        let area = properties.referenceAreaSquareFeet
        let ix = properties.axialInertia
        let iy = properties.transverseInertia

        // Evaluate differential rates dS/dt
        func computeDerivatives(s: State6DOF) -> Derivatives {
            let v = max(10.0, s.totalSpeedFPS)
            let mach = v / soundSpeedFPS
            let qDynamic = 0.5 * airDensitySlugFt3 * v * v

            // Aerodynamic derivatives at current Mach
            let cd0 = coefficients.cd0(mach)
            let clA = coefficients.clAlpha(mach)
            let cmA = coefficients.cmAlpha(mach)
            let cmq = coefficients.cmq(mach)
            let clp = coefficients.clp(mach)
            let cmag = coefficients.cMag(mach)

            // Velocity direction unit vector
            let uvx = s.vx / v
            let uvy = s.vy / v
            let uvz = s.vz / v

            // Pointing vector from Euler angles
            let cosP = cos(s.pitch)
            let sinP = sin(s.pitch)
            let cosY = cos(s.yaw)
            let sinY = sin(s.yaw)

            let px = cosP * cosY
            let py = sinP
            let pz = cosP * sinY

            // Angle of attack vector: delta = p_vec - uv_vec
            let deltax = px - uvx
            let deltay = py - uvy
            let deltaz = pz - uvz
            let alphaTotal = sqrt(deltax * deltax + deltay * deltay + deltaz * deltaz)

            // 1. Aerodynamic Drag Force: F_drag = -q * S * CD * uv_vec
            let cdTotal = cd0 + 1.5 * alphaTotal * alphaTotal
            let fDragMag = qDynamic * area * cdTotal
            let fDragX = -fDragMag * uvx
            let fDragY = -fDragMag * uvy
            let fDragZ = -fDragMag * uvz

            // 2. Aerodynamic Lift Force: F_lift = q * S * CL_alpha * delta_vec
            let fLiftMag = qDynamic * area * clA
            let fLiftX = fLiftMag * deltax
            let fLiftY = fLiftMag * deltay
            let fLiftZ = fLiftMag * deltaz

            // 3. Gravity Force
            let fGravY = -32.17405 * mass

            // 4. Magnus Force: F_mag = 0.5 * rho * S * d * Cmag * (omega_p x v_vec)
            let fMagFactor = 0.5 * airDensitySlugFt3 * area * diamFeet * cmag * s.p
            let fMagX = 0.0
            let fMagY = -fMagFactor * uvz
            let fMagZ = fMagFactor * uvy

            // Total linear accelerations
            let dvx = (fDragX + fLiftX + fMagX) / mass
            let dvy = (fDragY + fLiftY + fGravY + fMagY) / mass
            let dvz = (fDragZ + fLiftZ + fMagZ) / mass

            // Moments
            // Roll damping moment: M_p = q * S * d^2 * Clp * (p * d / 2v)
            let dp = (qDynamic * area * diamFeet * diamFeet * clp * (s.p * diamFeet / (2.0 * v))) / max(1e-9, ix)

            // Overturning moment: M_alpha = q * S * d * CM_alpha * delta
            let momAlphaMag = qDynamic * area * diamFeet * cmA
            let momPitch = momAlphaMag * deltay
            let momYaw = momAlphaMag * deltaz

            // Pitch/yaw damping moment: M_q = q * S * d^2 * (d / 2v) * Cmq * q
            let dampFactor = (qDynamic * area * diamFeet * diamFeet * (diamFeet / (2.0 * v)) * cmq)
            let dq = (momPitch + dampFactor * s.q - (ix - iy) * s.p * s.r) / max(1e-9, iy)
            let dr = (momYaw + dampFactor * s.r - (iy - ix) * s.p * s.q) / max(1e-9, iy)

            return Derivatives(
                dx: s.vx,
                dy: s.vy,
                dz: s.vz,
                dvx: dvx,
                dvy: dvy,
                dvz: dvz,
                dpitch: s.q,
                dyaw: s.r,
                droll: s.p,
                dp: dp,
                dq: dq,
                dr: dr
            )
        }

        func stepRK4(s: State6DOF, dt: Double) -> State6DOF {
            let k1 = computeDerivatives(s: s)

            let s2 = State6DOF(
                x: s.x + 0.5 * dt * k1.dx,
                y: s.y + 0.5 * dt * k1.dy,
                z: s.z + 0.5 * dt * k1.dz,
                vx: s.vx + 0.5 * dt * k1.dvx,
                vy: s.vy + 0.5 * dt * k1.dvy,
                vz: s.vz + 0.5 * dt * k1.dvz,
                pitch: s.pitch + 0.5 * dt * k1.dpitch,
                yaw: s.yaw + 0.5 * dt * k1.dyaw,
                roll: s.roll + 0.5 * dt * k1.droll,
                p: s.p + 0.5 * dt * k1.dp,
                q: s.q + 0.5 * dt * k1.dq,
                r: s.r + 0.5 * dt * k1.dr
            )
            let k2 = computeDerivatives(s: s2)

            let s3 = State6DOF(
                x: s.x + 0.5 * dt * k2.dx,
                y: s.y + 0.5 * dt * k2.dy,
                z: s.z + 0.5 * dt * k2.dz,
                vx: s.vx + 0.5 * dt * k2.dvx,
                vy: s.vy + 0.5 * dt * k2.dvy,
                vz: s.vz + 0.5 * dt * k2.dvz,
                pitch: s.pitch + 0.5 * dt * k2.dpitch,
                yaw: s.yaw + 0.5 * dt * k2.dyaw,
                roll: s.roll + 0.5 * dt * k2.droll,
                p: s.p + 0.5 * dt * k2.dp,
                q: s.q + 0.5 * dt * k2.dq,
                r: s.r + 0.5 * dt * k2.dr
            )
            let k3 = computeDerivatives(s: s3)

            let s4 = State6DOF(
                x: s.x + dt * k3.dx,
                y: s.y + dt * k3.dy,
                z: s.z + dt * k3.dz,
                vx: s.vx + dt * k3.dvx,
                vy: s.vy + dt * k3.dvy,
                vz: s.vz + dt * k3.dvz,
                pitch: s.pitch + dt * k3.dpitch,
                yaw: s.yaw + dt * k3.dyaw,
                roll: s.roll + dt * k3.droll,
                p: s.p + dt * k3.dp,
                q: s.q + dt * k3.dq,
                r: s.r + dt * k3.dr
            )
            let k4 = computeDerivatives(s: s4)

            return State6DOF(
                x: s.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
                y: s.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
                z: s.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
                vx: s.vx + (dt / 6.0) * (k1.dvx + 2.0 * k2.dvx + 2.0 * k3.dvx + k4.dvx),
                vy: s.vy + (dt / 6.0) * (k1.dvy + 2.0 * k2.dvy + 2.0 * k3.dvy + k4.dvy),
                vz: s.vz + (dt / 6.0) * (k1.dvz + 2.0 * k2.dvz + 2.0 * k3.dvz + k4.dvz),
                pitch: s.pitch + (dt / 6.0) * (k1.dpitch + 2.0 * k2.dpitch + 2.0 * k3.dpitch + k4.dpitch),
                yaw: s.yaw + (dt / 6.0) * (k1.dyaw + 2.0 * k2.dyaw + 2.0 * k3.dyaw + k4.dyaw),
                roll: s.roll + (dt / 6.0) * (k1.droll + 2.0 * k2.droll + 2.0 * k3.droll + k4.droll),
                p: s.p + (dt / 6.0) * (k1.dp + 2.0 * k2.dp + 2.0 * k3.dp + k4.dp),
                q: s.q + (dt / 6.0) * (k1.dq + 2.0 * k2.dq + 2.0 * k3.dq + k4.dq),
                r: s.r + (dt / 6.0) * (k1.dr + 2.0 * k2.dr + 2.0 * k3.dr + k4.dr)
            )
        }

        var sampleIndex = 0
        var nextSampleFeet = Double(sampleIndex) * stepFeet
        var t = 0.0

        func emitPoint(s: State6DOF, elapsed: Double, xReportFeet: Double) {
            let pathInches = s.y * 12.0
            let moaDrop = -Math.radToMOA(atan(s.y / max(xReportFeet, 1e-9)))
            let windageInches = s.z * 12.0
            let moaWindage = Math.radToMOA(atan((windageInches / 12.0) / max(xReportFeet, 1e-9)))
            let v = s.totalSpeedFPS
            let weightGrains = properties.weight.converted(to: .grains).value
            let ftlbs = weightGrains * (pow(v, 2)) / (2.0 * 32.163 * 7000.0)

            let duration = Measurement(value: elapsed, unit: UnitDuration.seconds)
            let rangeMeasurement = Measurement(value: Double(sampleIndex) * stepInPreferred.value, unit: ballistics.preferredDistanceUnit)

            // 6-DOF stability metrics
            let mach = v / soundSpeedFPS
            let cmA = coefficients.cmAlpha(mach)

            let qDyn = 0.5 * airDensitySlugFt3 * v * v
            let mOverturnPerRad = qDyn * area * diamFeet * cmA
            let sg = (ix * ix * s.p * s.p) / max(1e-9, 4.0 * iy * mOverturnPerRad)

            // Dynamic stability Sd
            let sd = max(0.01, min(2.0, 1.0 + 0.1 * (sg - 1.5)))

            let yawOfReposeRad = (8.0 * ix * s.p * 32.17405) / max(1e-9, diamFeet * airDensitySlugFt3 * area * pow(v, 3) * cmA)

            let point = Point(
                range: rangeMeasurement,
                drop: Measurement(value: pathInches, unit: .inches),
                dropCorrection: Measurement(value: moaDrop, unit: .minutesOfAngle),
                windage: Measurement(value: windageInches, unit: .inches),
                windageCorrection: Measurement(value: moaWindage, unit: .minutesOfAngle),
                travelTime: duration,
                velocity: Measurement(value: v, unit: .feetPerSecond),
                velocityX: Measurement(value: s.vx, unit: .feetPerSecond),
                velocityY: Measurement(value: s.vy, unit: .feetPerSecond),
                energy: Measurement(value: ftlbs, unit: .footPounds),
                spinDrift: Measurement(value: windageInches, unit: .inches),
                coriolisHorizontal: nil,
                coriolisVertical: nil,
                spinRateRPM: s.spinRateRPM,
                stabilityFactorSg: sg,
                dynamicStabilitySd: sd,
                yawOfReposeAngle: Measurement(value: yawOfReposeRad, unit: .radians)
            )

            ballistics.distances.append(point)
        }

        emitPoint(s: state, elapsed: t, xReportFeet: 0)
        sampleIndex += 1
        nextSampleFeet = Double(sampleIndex) * stepFeet

        while true {
            let v = max(10.0, state.totalSpeedFPS)
            let dt = 0.5 / v

            let nextState = stepRK4(s: state, dt: dt)

            while nextState.x >= nextSampleFeet {
                let alpha = (nextSampleFeet - state.x) / max(1e-9, nextState.x - state.x)
                let interpState = State6DOF(
                    x: nextSampleFeet,
                    y: state.y + alpha * (nextState.y - state.y),
                    z: state.z + alpha * (nextState.z - state.z),
                    vx: state.vx + alpha * (nextState.vx - state.vx),
                    vy: state.vy + alpha * (nextState.vy - state.vy),
                    vz: state.vz + alpha * (nextState.vz - state.vz),
                    pitch: state.pitch + alpha * (nextState.pitch - state.pitch),
                    yaw: state.yaw + alpha * (nextState.yaw - state.yaw),
                    roll: state.roll + alpha * (nextState.roll - state.roll),
                    p: state.p + alpha * (nextState.p - state.p),
                    q: state.q + alpha * (nextState.q - state.q),
                    r: state.r + alpha * (nextState.r - state.r)
                )
                let tInterp = t + alpha * dt
                emitPoint(s: interpState, elapsed: tInterp, xReportFeet: nextSampleFeet)

                sampleIndex += 1
                nextSampleFeet = Double(sampleIndex) * stepFeet
                if nextSampleFeet > maxFeet { break }
            }

            state = nextState
            t += dt

            if state.x >= maxFeet || state.totalSpeedFPS < 50.0 {
                break
            }
        }

        return ballistics
    }
}
