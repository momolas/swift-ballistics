//
//  Angle.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

public struct Angle: Sendable, Equatable, Hashable {

    /**
     Calculates the angle of elevation required to zero a firearm at a specific range.

     This method determines the angle of elevation that aligns the projectile's trajectory
     with the point of aim at the zero range, considering the drag function, drag coefficient, initial velocity,
     sight height, and the y-intercept (height offset at the muzzle or near the firearm).
    
     - Parameters:
       - dragFunction: The drag function (.g1 or .g7). Default is .g1.
       - dragCoefficient: The drag coefficient of the projectile.
       - initialVelocity: The muzzle velocity of the projectile.
       - sightHeight: The height of the sight above the bore axis.
       - zeroRange: The desired zero range where the projectile intersects the line of sight.
       - yIntercept: The vertical offset of the projectile at the muzzle in feet.

     - Returns:
       A `Double` representing the required angle of elevation in degrees to achieve the zero range.
    */
    static func zeroAngle(
        dragFunction: DragFunction = .g1,
        dragCoefficient: Double,
        initialVelocity: Measurement<UnitSpeed>,
        sightHeight: Measurement<UnitLength>,
        zeroRange: Measurement<UnitLength>,
        yIntercept: Double
    ) -> Double {

        // Numerical Integration variables
        var dt: Double = 1 / initialVelocity.converted(to: .feetPerSecond).value // The solution accuracy generally doesn't suffer if within a foot for each second of time.
        var y: Double = -sightHeight.converted(to: .inches).value / 12
        var x: Double = 0
        var da: Double // Change in the bore angle used to iterate toward the correct zero angle.

        // State variables for each integration loop
        let initialVelocityFPS = initialVelocity.converted(to: .feetPerSecond).value
        var v: Double = 0
        var vx: Double = 0
        var vy: Double = 0
        var vx1: Double = 0
        var vy1: Double = 0 // Last frame's velocity, used for computing average velocity.
        var dv: Double = 0
        var dvx: Double = 0
        var dvy: Double = 0 // Acceleration
        var Gx: Double = 0
        var Gy: Double = 0 // Gravitational acceleration

        var angle: Double = 0 // The actual angle of the bore.
        var quit = false // We know it's time to quit our successive approximation loop when this is true.

        // Start with a very coarse angular change to quickly solve even large launch angle problems.
        da = Math.degToRad(14)

        // Successive approximation of the bore angle
        while !quit {
            vy = initialVelocityFPS * sin(angle)
            vx = initialVelocityFPS * cos(angle)
            Gx = Constants.GRAVITY * sin(angle)
            Gy = Constants.GRAVITY * cos(angle)
            
            x = 0
            y = -sightHeight.converted(to: .inches).value / 12

            while x <= zeroRange.converted(to: .yards).value * 3 {
                vy1 = vy
                vx1 = vx
                v = sqrt(vx * vx + vy * vy)
                dt = 1 / v

                dv = Drag.retard(dragFunction: dragFunction, dragCoefficient: dragCoefficient, projectileVelocity: v)
                dvy = -dv * vy / v * dt
                dvx = -dv * vx / v * dt

                vx += dvx + dt * Gx
                vy += dvy + dt * Gy

                x += dt * (vx + vx1) / 2
                y += dt * (vy + vy1) / 2

                // Break early to save CPU time if we won't find a solution
                if vy < 0 && y < yIntercept {
                    break
                }
                if vy > 3 * vx {
                    break
                }
            }

            // Adjust the angle for successive approximation
            if y > yIntercept && da > 0 {
                da = -da / 2
            }

            if y < yIntercept && da < 0 {
                da = -da / 2
            }

            // Check stopping conditions
            if abs(da) < Math.moaToRad(0.01) || angle > Math.degToRad(45) {
                quit = true
            }

            angle += da
        }

        return Math.radToDeg(angle) // Convert to degrees for the return value.
    }
}
