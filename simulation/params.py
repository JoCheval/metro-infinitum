
###############################################################################
#                               Wagon Specs                                   #
###############################################################################

tunnel_depth = 0  # Depth of tunnels
# ground level at mtl (m) http://dateandtime.info/fr/citycoordinates.php?id=6077243   &   https://rechneronline.de/earth-radius/
earth_radius = 6367483

azur_train_lenght = 152.437  # Longueur des trains/stations actuelles (m)

largeur_wagon = 2.5
hauteur_wagon = 3.7142
longeur_wagon = 16.8

vitesse_max = 72 * 1000 / 3600  # m/s
accel_max = 1.203   # m/s2
deccel_nom = 1.230  # m/s2
deccel_max = 1.790  # m/s2
max_jerk = 0.60     # m/s3 while accelerating/decelerating

max_freestanding_jerk = 0.60  # m/s3
max_freestanding_accel = 0.93  # m/s2

masse_wagon = 26990  # kg

w_per_train = 3     # Wagons per train

###############################################################################
#                             Operation Params                                #
###############################################################################

stopped_max_speed = 3 / 3.6  # 3km/h

# Side doors
open_max_speed = 1 / 3.6  # 1km/h
side_doors_openning_time = 2
side_doors_closing_time = 2
min_time_side_doors_open = 10
max_dist_open_doors = 1  # Max distance from destination to open doors
side_doors_error_prob = 0.0  # Probability to have problem closing side doors
side_doors_deblock_prob = 0.5  # Probability that retry works

# Boa doors
boa_openning_time = 2
boa_closing_time = 2
full_boa_time = 40    # temps durant lequel les wagons communiquent
boa_error_prob = 0.0  # Probability to have problem closing boa doors
boa_deblock_prob = 0.5  # Probability that retry works

# Rendez vous
side_doors_standby_time = 2   # Min time after side doors closed before accel
delta_rv_speed = 8 / 3.6   # delta vitesse rendez vous
time_at_rv_speed = 15    # temps a vitesse rendez vous
min_dist_couple = 0.2   # distance minimale pour un acouplement
coupling_time = 1

# Wait time
min_wait_time_first_train = 40   # Temps minimum avant qu'un nouveau train partent

breaking_marjin = 0

decoupling_time = 1

wrong_velocity_threshold = 1 / 3.6  # Max velocity error tolerated (m/s)

dist_before_station_set_velocity = 150
