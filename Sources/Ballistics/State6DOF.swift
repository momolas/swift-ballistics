//
//  State6DOF.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

/// Represents the 12-dimensional state vector of a rigid-body projectile at a discrete time instant in 6-DOF simulations.
public struct State6DOF: Sendable, Equatable, Hashable {

    // MARK: - Position (Earth frame, in feet)
    public var x: Double
    public var y: Double
    public var z: Double

    // MARK: - Linear Velocity (Earth frame, in ft/s)
    public var vx: Double
    public var vy: Double
    public var vz: Double

    // MARK: - Angular Orientation (Euler angles in radians: Pitch, Yaw, Roll)
    public var pitch: Double
    public var yaw: Double
    public var roll: Double

    // MARK: - Angular Rates (Body frame, in rad/s: Roll p, Pitch q, Yaw r)
    public var p: Double
    public var q: Double
    public var r: Double

    // MARK: - Computed Properties

    /// Total linear speed V = sqrt(vx^2 + vy^2 + vz^2) in ft/s.
    public var totalSpeedFPS: Double {
        sqrt(vx * vx + vy * vy + vz * vz)
    }

    /// Spin rate in revolutions per minute (RPM).
    public var spinRateRPM: Double {
        (p * 60.0) / (2.0 * Double.pi)
    }

    public init(
        x: Double = 0,
        y: Double = 0,
        z: Double = 0,
        vx: Double,
        vy: Double,
        vz: Double = 0,
        pitch: Double,
        yaw: Double = 0,
        roll: Double = 0,
        p: Double,
        q: Double = 0,
        r: Double = 0
    ) {
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
        self.p = p
        self.q = q
        self.r = r
    }
}
