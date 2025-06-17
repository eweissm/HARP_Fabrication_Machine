import configparser
import numpy as np
import math
from scipy.interpolate import interp1d

class Muscle:
    def __init__(self, config_path):
        config = configparser.ConfigParser()
        config.read(config_path)
        params = config['Muscle Parameters']

        # Convert all parameters from the config to float and store as attributes
        for key, value in params.items():
            setattr(self, key, float(value))

        self.g = 9.81         # gravitational acceleration

    def _harp_model(self, p_array):
        # Constants
        G_nylon = self.e_n / (2 * (1 + self.nu_n))
        G_silicone = self.e_s / (2 * (1 + self.nu_s))
        beta = 1 / (self.e_s * (self.ro**2 - self.ri**2))

        # Displacement function u(r)
        def u(r): return self.ri**2 * p_array * r * beta * ((1 - self.nu_s) + (1 + self.nu_s) * self.ro**2 / r**2)

        ro_p = self.ro + u(self.ro)
        ri_p = self.ri + u(self.ri)

        C = self.r_coil / ro_p

        alpha = np.arctan2(2 * math.pi * self.ro, self.pitch)
        alpha_c = np.arctan2(2 * math.pi * self.r_coil, self.l0 / self.n)

        lambda1 = -self.nu_s * beta * self.ri**2 * p_array + 1
        lambda2 = 2 * beta * self.ri**2 * p_array + 1

        epsilon_p = C * np.tan(alpha_c) * np.tan(alpha) * (
            np.sqrt((1 - lambda1**2 * np.cos(alpha)**2) / (lambda2**2 * np.sin(alpha)**2)) - 1
        )

        K_sigma = (
            (G_silicone * (2 * ro_p)**4 + G_nylon * self.d_f**4)
            / (32 * self.r_coil**2 * (ro_p**2 - ri_p**2) * np.tan(alpha_c))
        )

        return K_sigma, epsilon_p

    def strain_lookup(self, p_min=0, p_max=40, step=0.1, force_N=0.1 * 9.81):
        # Pressure array in Pascals (1 psi = 6894.76 Pa)
        p_psi = np.arange(p_min, p_max + step, step)
        p_pa = p_psi * 6894.76

        # Cross-sectional area
        A = math.pi * (self.ro**2 - self.ri**2)

        # Get stiffness and pre-strain
        K_sigma, epsilon_p = self._harp_model(p_pa)

        # Stress
        sigma = force_N / A

        # Strain computation
        strain = -(sigma / K_sigma + epsilon_p)
        strain = strain-strain[0]
        # Return lookup table as dictionary: pressure (psi) → strain
        return dict(zip(strain, p_psi))



    def pressure_from_strain(self, strain_input, lookup_table=None):
        """
        Interpolates to find the pressure (in psi) corresponding to a given strain.
        If lookup_table is not provided, it generates one using default settings.
        """
        if lookup_table is None:
            lookup_table = self.strain_lookup()

        # Unpack pressures and strains
        pressures = np.array(list(lookup_table.keys()))
        strains = np.array(list(lookup_table.values()))

        # Check if strain is within range
        if not (np.min(strains) <= strain_input <= np.max(strains)):
            raise ValueError(f"Strain {strain_input} is outside the interpolation range "
                             f"({np.min(strains):.6f} to {np.max(strains):.6f})")

        # Interpolation: strain → pressure
        interpolator = interp1d(strains, pressures, kind='linear', fill_value="extrapolate")
        pressure_estimate = float(interpolator(strain_input))

        return pressure_estimate
