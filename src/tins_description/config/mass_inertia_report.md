SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED

Simulation reference values, not a weighed or physically qualified assembly.

Vehicle without aircraft: **385.119249 kg**. This consists of the
unverified 295 kg crawler/battery catalogue baseline and 90.119249 kg
of current mounting/mechanism estimates, including the single 4.54 kg motor/brake lump.
Park CG in base_link: [0.027665364937316216, 0.00040817844642192745, 0.36940780216701186] m. Estimated shift from the assumed crawler
baseline CG: [-0.008451464098909914, -0.00012469401699933364, 0.0830356655773648] m. See physical_mass_rules.json for material, catalogue,
missing-internal-mass and battery-distribution assumptions.

The eight separate aircraft use a 0.5 kg catalogue reference each (4 kg total),
compared with the unchanged 16 kg design allocation. Their exact supplied-build
mass and CG are unmeasured. They add no mass to the vehicle URDF. Static display
aircraft are massless. The 16 kg alternative remains a load case, not a mass credit.

mass_inertia_report.json contains every physical link tensor and 130 load cases
(26 waypoint states × five occupancy/allocation cases). Aircraft box estimates
are for this load comparison only. No OEM CG-limit, stability, strength, locking,
braking, power-loss or flight check has passed on the basis of this calculation.
