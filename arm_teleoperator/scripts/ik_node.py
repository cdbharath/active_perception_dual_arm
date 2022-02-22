#!/usr/bin/env python

from kinova_ik.kinova_gen3_ik import KinovaGen3IK

if __name__ == "__main__":
    kinova_ik = KinovaGen3IK()
    print(kinova_ik.solve_ik([0.0, 0.0, 1.00], 
                            [0, 0, 0])) #zyx

