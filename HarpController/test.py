from MuscleClass import Muscle
from scipy.interpolate import interp1d
import numpy as np


global pressures_lookup, strains_lookup

muscle = Muscle("MuscleConfig.ini")
lookup_table = muscle.strain_lookup()
strains_lookup = np.array(list(lookup_table.keys()))
pressures_lookup = np.array(list(lookup_table.values()))
print(pressures_lookup)
print(strains_lookup)
# Example: print strain at specific pressures
for strain, pressure in lookup_table.items():
    print(f"Strain: {strain:.6f} → Pressure: {pressure:.1f} psi")

def pressure_from_strain( strain_input):
    """
    Interpolates to find the pressure (in psi) corresponding to a given strain.
    """

    global pressures_lookup, strains_lookup


    # Check if strain is within range
    if not (np.min(strains_lookup) <= strain_input <= np.max(strains_lookup)):
        raise ValueError(f"Strain {strain_input} is outside the interpolation range "
                         f"({np.min(strains_lookup):.6f} to {np.max(strains_lookup):.6f})")

    # Interpolation: strain → pressure
    interpolator = interp1d(strains_lookup, pressures_lookup, kind='linear', fill_value="extrapolate")
    pressure_estimate = float(interpolator(strain_input))

    return pressure_estimate

print(pressure_from_strain(.1))
