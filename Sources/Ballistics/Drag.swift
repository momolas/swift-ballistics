//
//  Drag.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

struct Drag {

    /**
     Calculates the drag coefficient (Cd) for a given projectile velocity and atmospheric conditions.
     - Parameters:
        - model: The drag model to use (e.g., .g1, .g7).
        - velocity: The projectile's velocity in meters per second.
        - atmosphere: The atmospheric conditions.
     - Returns: The calculated drag coefficient (Cd).
    */
    static func coefficient(for model: DragModel, velocity: Double, atmosphere: Atmosphere) -> Double {
        let dragTable: [DragTable]
        switch model {
        case .g1:
            dragTable = g1DragTable
        case .g7:
            dragTable = g7DragTable
        }

        let speedOfSound = atmosphere.speedOfSound()
        guard speedOfSound > 0 else { return 0 }

        let machValue = velocity / speedOfSound

        // Find the two points in the table that bracket the mach value
        guard let upperIndex = dragTable.firstIndex(where: { $0.mach >= machValue }) else {
            // If mach value is higher than table, return the last value
            return dragTable.last?.cd ?? 0
        }

        if upperIndex == 0 {
            return dragTable.first?.cd ?? 0
        }

        let lowerIndex = upperIndex - 1
        let lowerPoint = dragTable[lowerIndex]
        let upperPoint = dragTable[upperIndex]

        // Linear interpolation
        let machRange = upperPoint.mach - lowerPoint.mach
        guard machRange > 0 else { return lowerPoint.cd }

        let cdRange = upperPoint.cd - lowerPoint.cd
        let machFraction = (machValue - lowerPoint.mach) / machRange

        let interpolatedCd = lowerPoint.cd + (cdRange * machFraction)

        return interpolatedCd
    }
  
    /**
     Calculates the deceleration of a projectile due to drag.
     - Parameters:
        - dragCoefficient: The projectile's drag coefficient.
        - projectileVelocity: The projectile's velocity.
     - Returns: The deceleration value.
    */
    static func retard(dragCoefficient: Double, projectileVelocity: Double) -> Double {
        return -dragCoefficient * projectileVelocity * projectileVelocity
    }
}
