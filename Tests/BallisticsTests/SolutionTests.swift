//
//  SolutionTests.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 05/12/2024.
//

import Foundation
import Testing
import Ballistics
import Numerics

@Test func g1Solution() async throws {
    // G1 Test case based on Hornady .308 168gr BTHP Match
    // G1 BC = 0.450
    // Muzzle Velocity = 2700 ft/s
    let solution = Ballistics.solve(
        dragModel: .g1,
        ballisticCoefficient: 0.450,
        initialVelocity: Measurement(value: 2700, unit: .feetPerSecond),
        sightHeight: Measurement(value: 1.5, unit: .inches),
        shootingAngle: Measurement(value: 0, unit: .degrees),
        zeroRange: Measurement(value: 100, unit: .yards),
        atmosphere: Atmosphere(), // Standard atmosphere
        windSpeed: Measurement(value: 10, unit: .milesPerHour),
        windAngle: 90,
        weight: Measurement(value: 168, unit: .grains)
    )

    let point300 = try #require(solution.getPoint(at: Measurement(value: 300, unit: .yards)))

    // Values from Hornady's online calculator
    #expect(point300.dropMeasurement.converted(to: .inches).value.isApproximatelyEqual(to: -13.9, absoluteTolerance: 0.2))
    #expect(point300.windageMeasurement.converted(to: .inches).value.isApproximatelyEqual(to: 7.8, absoluteTolerance: 0.2))
    #expect(point300.velocityMeasurement.converted(to: .feetPerSecond).value.isApproximatelyEqual(to: 2146, absoluteTolerance: 5))
}

@Test func g7SolutionWithClicks() async throws {
    // G7 Test case based on Berger .308 180gr Hybrid Target
    // G7 BC = 0.316
    // Muzzle Velocity = 2850 ft/s
    // Scope clicks = 1/4 MOA

    let solution = Ballistics.solve(
        dragModel: .g7,
        ballisticCoefficient: 0.316,
        initialVelocity: Measurement(value: 2850, unit: .feetPerSecond),
        sightHeight: Measurement(value: 1.5, unit: .inches),
        shootingAngle: Measurement(value: 0, unit: .degrees),
        zeroRange: Measurement(value: 100, unit: .yards),
        atmosphere: Atmosphere(), // Standard atmosphere
        windSpeed: Measurement(value: 10, unit: .milesPerHour),
        windAngle: 90,
        weight: Measurement(value: 180, unit: .grains),
        scopeClickValue: 0.25
    )

    let point500 = try #require(solution.getPoint(at: Measurement(value: 500, unit: .yards)))

    // Values from JBM Ballistics online calculator
    #expect(point500.dropMeasurement.converted(to: .inches).value.isApproximatelyEqual(to: -53.4, absoluteTolerance: 0.5))
    #expect(point500.windageMeasurement.converted(to: .inches).value.isApproximatelyEqual(to: 18.9, absoluteTolerance: 0.5))

    // Clicks = round(MOA / 0.25)
    // Drop MOA at 500yd = 10.2 MOA -> -41 clicks
    // Windage MOA at 500yd = 3.61 MOA -> 14 clicks
    #expect(try #require(point500.dropCorrectionClicks) == -41)
    #expect(try #require(point500.windageCorrectionClicks) == 14)
}

@Test func interpolationTest() async throws {
    // Re-use the G1 solution to test interpolation
    let solution = Ballistics.solve(
        dragModel: .g1,
        ballisticCoefficient: 0.450,
        initialVelocity: Measurement(value: 2700, unit: .feetPerSecond),
        sightHeight: Measurement(value: 1.5, unit: .inches),
        shootingAngle: Measurement(value: 0, unit: .degrees),
        zeroRange: Measurement(value: 100, unit: .yards),
        atmosphere: Atmosphere(),
        windSpeed: Measurement(value: 10, unit: .milesPerHour),
        windAngle: 90,
        weight: Measurement(value: 168, unit: .grains)
    )

    // Get points at 200 and 300 yards to use for manual interpolation
    let p200 = try #require(solution.getPoint(at: Measurement(value: 200, unit: .yards)))
    let p300 = try #require(solution.getPoint(at: Measurement(value: 300, unit: .yards)))

    // Get the interpolated point at 250 yards
    let p250 = try #require(solution.getPoint(at: Measurement(value: 250, unit: .yards)))

    // Manually interpolate the drop and windage
    let expectedDrop = (p200.drop + p300.drop) / 2.0
    let expectedWindage = (p200.windage + p300.windage) / 2.0

    // Check that the interpolated point's values are correct
    #expect(p250.drop.isApproximatelyEqual(to: expectedDrop, absoluteTolerance: 0.01))
    #expect(p250.windage.isApproximatelyEqual(to: expectedWindage, absoluteTolerance: 0.01))
    #expect(p250.dropCorrectionClicks == nil) // Clicks should be nil for interpolated points
    #expect(p250.windageCorrectionClicks == nil)
}
