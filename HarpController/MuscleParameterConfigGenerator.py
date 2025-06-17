import configparser

config = configparser.ConfigParser()

# Adding sections and key-value pairs
config['Muscle Parameters'] = {'ro':  5.5/2000,
                               'ri':  2.6/2000,
                               'E_n': 916*10**6,
                               'nu_n': .25,
                               'nu_s': .3,
                               'E_s': 2.5*10**6,
                               'd_f': .032*25.4/1000,
                               'r_coil': 7.4/2000,
                               'n': 20,
                               'pitch': 12/1000,
                               'L_noLoad': 155/1000
                }

# Writing to a file
with open('MuscleConfig.ini', 'w') as configfile:
    config.write(configfile)