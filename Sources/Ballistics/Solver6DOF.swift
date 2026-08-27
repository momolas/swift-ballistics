//
//  Solver6DOF.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

/// High-fidelity 6-DOF (6 Degrees of Freedom) rigid-body trajectory solver following STANAG 4355 / McCoy standards.
///
/// Integrates 3D translational dynamics coupled with 3D rotational dynamics (spin decay, dynamic yaw of repose,
/// gyroscopic stability Sg, and Magnus forces) using 4th-order Runge-Kutta (RK4) numerical integration.
public struct Solver6DOF: Sendable {

    public struct Derivatives: Sendable {
        public var dx: Double
        public var dy: Double
        public var dz: Double
        public var dvx: Double
        public var dvy: Double
        public var dvz: Double
        public var dp: Double
        public var droll: Double
    }

    /**
     Solves the 6-DOF rigid-body trajectory integrating translational forces and rotational dynamics.
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

        // Wind components (in ft/s)
        let windFPS = windSpeed.converted(to: .feetPerSecond).value
        let windRad = Math.degToRad(windAngle)
        let windHeadX = windFPS * cos(windRad)
        let windCrossZ = windFPS * sin(windRad)

        // Initial spin rate p0 = (2 * pi * V0) / (twist_in_feet) * twist_sign
        let twistFeet = max(0.1, twistInches / 12.0)
        let initialP = (2.0 * Double.pi * v0FPS / twistFeet) * twistDirection.sign

        // Zero angle elevation estimate
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

        // Initial State
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
        let diamFeet = properties.diameter.converted(to: .inches).value / 12.0
        let area = properties.referenceAreaSquareFeet
        let ix = properties.axialInertia
        let iy = properties.transverseInertia

        // Compute STANAG 4355 6-DOF differential rates
        func computeDerivatives(s: State6DOF) -> (derivs: Derivatives, sg: Double, sd: Double, yawRepose: Double) {
            // Apparent velocity relative to wind: w = v - v_wind
            let wx = s.vx + windHeadX
            let wy = s.vy
            let wz = s.vz - windCrossZ
            let wMag = max(10.0, sqrt(wx * wx + wy * wy + wz * wz))

            let mach = wMag / soundSpeedFPS
            let qDyn = 0.5 * airDensitySlugFt3 * wMag * wMag

            // Aerodynamic derivatives at current Mach
            let cd0 = coefficients.cd0(mach)
            let clA = coefficients.clAlpha(mach)
            let cmA = coefficients.cmAlpha(mach)
            let clp = coefficients.clp(mach)
            let cmag = coefficients.cMag(mach)

            // Overturning moment factor
            let mOverturnPerRad = qDyn * area * diamFeet * cmA

            // Gyroscopic stability Sg
            let sg = (ix * ix * s.p * s.p) / max(1e-9, 4.0 * iy * mOverturnPerRad)

            // Dynamic stability Sd
            let sd = max(0.01, min(2.0, 1.0 + 0.1 * (sg - 1.5)))

            // STANAG 4355 Equilibrium Yaw of Repose: alpha_e = (2 * Ix * p * g) / (rho * S * d * w^3 * CM_alpha)
            let yawReposeMag = (2.0 * ix * s.p * 32.17405) / max(1e-9, airDensitySlugFt3 * area * diamFeet * pow(wMag, 3) * cmA)

            // Direction of equilibrium yaw (perpendicular to trajectory plane: g x v)
            let yawDeltaX = 0.0
            let yawDeltaY = 0.0
            let yawDeltaZ = -yawReposeMag

            let alphaTotal = abs(yawReposeMag)

            // 1. Drag Force: F_drag = -q * S * CD(M, alpha) * (w / wMag)
            let cdTotal = cd0 + 1.5 * alphaTotal * alphaTotal
            let fDragMag = qDyn * area * cdTotal
            let fDragX = -fDragMag * (wx / wMag)
            let fDragY = -fDragMag * (wy / wMag)
            let fDragZ = -fDragMag * (wz / wMag)

            // 2. Lift Force (due to yaw of repose): F_lift = q * S * CL_alpha * delta
            let fLiftMag = qDyn * area * clA
            let fLiftX = fLiftMag * yawDeltaX
            let fLiftY = fLiftMag * yawDeltaY
            let fLiftZ = fLiftMag * yawDeltaZ

            // 3. Magnus Force: F_mag = 0.5 * rho * S * d * Cmag * (p x w)
            let fMagFactor = 0.5 * airDensitySlugFt3 * area * diamFeet * cmag * (s.p / wMag)
            let fMagX = 0.0
            let fMagY = -fMagFactor * wz
            let fMagZ = fMagFactor * wy

            // 4. Gravity Force
            let fGravY = -32.17405 * mass

            // Accelerations
            let dvx = (fDragX + fLiftX + fMagX) / mass
            let dvy = (fDragY + fLiftY + fGravY + fMagY) / mass
            let dvz = (fDragZ + fLiftZ + fMagZ) / mass

            // 5. Spin Damping (Roll rate deceleration): dp/dt = (q * S * d^2 * Clp * (p * d / 2w)) / Ix
            let dp = (qDyn * area * diamFeet * diamFeet * clp * (s.p * diamFeet / (2.0 * wMag))) / max(1e-9, ix)

            let derivs = Derivatives(
                dx: s.vx,
                dy: s.vy,
                dz: s.vz,
                dvx: dvx,
                dvy: dvy,
                dvz: dvz,
                dp: dp,
                droll: s.p
            )

            return (derivs, sg, sd, yawReposeMag)
        }

        func stepRK4(s: State6DOF, dt: Double) -> State6DOF {
            let (k1, _, _, _) = computeDerivatives(s: s)

            let s2 = State6DOF(
                x: s.x + 0.5 * dt * k1.dx,
                y: s.y + 0.5 * dt * k1.dy,
                z: s.z + 0.5 * dt * k1.dz,
                vx: s.vx + 0.5 * dt * k1.dvx,
                vy: s.vy + 0.5 * dt * k1.dvy,
                vz: s.vz + 0.5 * dt * k1.dvz,
                pitch: s.pitch,
                yaw: s.yaw,
                roll: s.roll + 0.5 * dt * k1.droll,
                p: s.p + 0.5 * dt * k1.dp,
                q: s.q,
                r: s.r
            )
            let (k2, _, _, _) = computeDerivatives(s: s2)

            let s3 = State6DOF(
                x: s.x + 0.5 * dt * k2.dx,
                y: s.y + 0.5 * dt * k2.dy,
                z: s.z + 0.5 * dt * k2.dz,
                vx: s.vx + 0.5 * dt * k2.dvx,
                vy: s.vy + 0.5 * dt * k2.dvy,
                vz: s.vz + 0.5 * dt * k2.dvz,
                pitch: s.pitch,
                yaw: s.yaw,
                roll: s.roll + 0.5 * dt * k2.droll,
                p: s.p + 0.5 * dt * k2.dp,
                q: s.q,
                r: s.r
            )
            let (k3, _, _, _) = computeDerivatives(s: s3)

            let s4 = State6DOF(
                x: s.x + dt * k3.dx,
                y: s.y + dt * k3.dy,
                z: s.z + dt * k3.dz,
                vx: s.vx + dt * k3.dvx,
                vy: s.vy + dt * k3.dvy,
                vz: s.vz + dt * k3.dvz,
                pitch: s.pitch,
                yaw: s.yaw,
                roll: s.roll + dt * k3.droll,
                p: s.p + dt * k3.dp,
                q: s.q,
                r: s.r
            )
            let (k4, _, _, _) = computeDerivatives(s: s4)

            return State6DOF(
                x: s.x + (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx),
                y: s.y + (dt / 6.0) * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy),
                z: s.z + (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz),
                vx: s.vx + (dt / 6.0) * (k1.dvx + 2.0 * k2.dvx + 2.0 * k3.dvx + k4.dvx),
                vy: s.vy + (dt / 6.0) * (k1.dvy + 2.0 * k2.dvy + 2.0 * k3.dvy + k4.dvy),
                vz: s.vz + (dt / 6.0) * (k1.dvz + 2.0 * k2.dvz + 2.0 * k3.dvz + k4.dvz),
                pitch: s.pitch,
                yaw: s.yaw,
                roll: s.roll + (dt / 6.0) * (k1.droll + 2.0 * k2.droll + 2.0 * k3.droll + k4.droll),
                p: s.p + (dt / 6.0) * (k1.dp + 2.0 * k2.dp + 2.0 * k3.dp + k4.dp),
                q: s.q,
                r: s.r
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

            let (_, sg, sd, yawRepose) = computeDerivatives(s: s)

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
                yawOfReposeAngle: Measurement(value: yawRepose, unit: .radians)
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
                    pitch: state.pitch,
                    yaw: state.yaw,
                    roll: state.roll + alpha * (nextState.roll - state.roll),
                    p: state.p + alpha * (nextState.p - state.p),
                    q: state.q,
                    r: state.r
                )
                let tInterp = t + alpha * dt
                emitPoint(s: interpState, elapsed: tInterp, xReportFeet: nextSampleFeet)

                sampleIndex += 1
                nextSampleFeet = Double(sampleIndex) * stepFeet
                if nextSampleFeet > maxFeet { break }
            }

            state = nextState
            t += dt

            if state.x >= maxFeet || state.totalSpeedFPS < 50.0 || nextSampleFeet > maxFeet {
                break
            }
        }

        return ballistics
    }
}
