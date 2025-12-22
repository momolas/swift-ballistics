//
//  Point.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

/// Represents a specific point along the trajectory of a projectile.
///
/// This struct captures various ballistic data at a given range, including position, velocity, energy, and timing.
public struct Point: Equatable, Hashable {

    /// The distance from the muzzle to this point.
    public let range: Measurement<UnitLength>

    /// The vertical drop of the projectile at this point relative to the line of sight.
    public let drop: Measurement<UnitLength>

    /// The angular correction required to compensate for the drop.
    public let dropCorrection: Measurement<UnitAngle>

    /// The horizontal drift of the projectile due to wind.
    public let windage: Measurement<UnitLength>

    public let windageCorrection: Measurement<UnitAngle>

    /// The travel time as a Measurement unit.
    public let travelTime: Measurement<UnitDuration>

    /// The total velocity of the projectile at this point.
    public let velocity: Measurement<UnitSpeed>

    /// The horizontal component of the velocity.
    public let velocityX: Measurement<UnitSpeed>

    /// The vertical component of the velocity.
    public let velocityY: Measurement<UnitSpeed>

    /// The kinetic energy of the projectile at this point.
    public let energy: Measurement<UnitEnergy>
}
