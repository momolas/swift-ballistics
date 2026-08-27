//
//  AdvancedEffectsTests.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation
import Testing
@testable import Ballistics

@Test func spinDriftCalculation() {
    // 175gr .308 projectile (diameter 0.308", length 1.24", 1:10" twist)
    let sg = SpinDrift.stabilityFactor(
        weight: Measurement(value: 175, unit: .grains),
        diameter: Measurement(value: 0.308, unit: .inches),
        length: Measurement(value: 1.24, unit: .inches),
        twist: Measurement(value: 10, unit: .inches),
        muzzleVelocity: Measurement(value: 2600, unit: .feetPerSecond)
    )

    // Sg should typically be around 1.5 - 2.6 for standard .308 175gr in 1:10 twist
    #expect(sg > 1.3 && sg < 3.0)

    // Spin drift for 1.0 second flight time (approx 600-700 yards)
    let rightDrift = SpinDrift.deflection(
        timeOfFlight: Measurement(value: 1.0, unit: .seconds),
        stabilityFactor: sg,
        twistDirection: .right
    )

    let leftDrift = SpinDrift.deflection(
        timeOfFlight: Measurement(value: 1.0, unit: .seconds),
        stabilityFactor: sg,
        twistDirection: .left
    )

    #expect(rightDrift.converted(to: .inches).value > 0)
    #expect(leftDrift.converted(to: .inches).value < 0)
    #expect(abs(rightDrift.converted(to: .inches).value + leftDrift.converted(to: .inches).value) < 1e-6)

    // At 1.0s, deflection is around 3.5 - 4.5 inches
    #expect(rightDrift.converted(to: .inches).value > 2.0 && rightDrift.converted(to: .inches).value < 6.0)
}

@Test func coriolisCalculation() {
    // Shooting at 45° North Latitude, East (90° Azimuth), 1000 yards (3000 ft), 1.5s time of flight
    let coriolis = Coriolis.deflection(
        latitude: Measurement(value: 45, unit: .degrees),
        azimuth: Measurement(value: 90, unit: .degrees),
        range: Measurement(value: 1000, unit: .yards),
        timeOfFlight: Measurement(value: 1.5, unit: .seconds)
    )

    // In Northern hemisphere (latitude > 0), horizontal Coriolis drift is positive (to the right)
    #expect(coriolis.horizontal.converted(to: .inches).value > 0)

    // Shooting East (azimuth = 90°), Eötvös effect causes bullet to hit high (positive vertical deflection)
    #expect(coriolis.vertical.converted(to: .inches).value > 0)

    // Shooting West (azimuth = 270°), Eötvös effect causes bullet to hit low (negative vertical deflection)
    let coriolisWest = Coriolis.deflection(
        latitude: Measurement(value: 45, unit: .degrees),
        azimuth: Measurement(value: 270, unit: .degrees),
        range: Measurement(value: 1000, unit: .yards),
        timeOfFlight: Measurement(value: 1.5, unit: .seconds)
    )

    #expect(coriolisWest.vertical.converted(to: .inches).value < 0)
}
