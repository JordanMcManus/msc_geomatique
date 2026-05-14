import unittest
import dpkt
import struct
import numpy as np
import numpy.testing as npt
from pathlib import Path
import scipy.io as sio
import laspy
from pyproj import Transformer

SBET_FIELD_NAMES = (
    "time",
    "latitude",
    "longitude",
    "altitude",
    "x_velocity",
    "y_velocity",
    "z_velocity",
    "roll",
    "pitch",
    "platform_heading",
    "wander_angle",
    "x_acceleration",
    "y_acceleration",
    "z_acceleration",
    "x_angular_rate",
    "y_angular_rate",
    "z_angular_rate",
)

LGF_SBET_FIELD_NAMES = (
    "time",
    "lgf_x",
    "lgf_y",
    "lgf_z",
    "x_velocity",
    "y_velocity",
    "z_velocity",
    "roll",
    "pitch",
    "platform_heading",
    "wander_angle",
    "x_acceleration",
    "y_acceleration",
    "z_acceleration",
    "x_angular_rate",
    "y_angular_rate",
    "z_angular_rate",
)

# SBET structured dtype (standard format)
SBET_DTYPE = np.dtype([
    ('time',   np.float64),
    ('lat',    np.float64),
    ('lon',    np.float64),
    ('alt',    np.float64),
    ('x_vel',  np.float64),
    ('y_vel',  np.float64),
    ('z_vel',  np.float64),
    ('roll',   np.float64),
    ('pitch',  np.float64),
    ('heading',np.float64),
    ('wander', np.float64),
    ('x_accel',np.float64),
    ('y_accel',np.float64),
    ('z_accel',np.float64),
    ('x_rate', np.float64),
    ('y_rate', np.float64),
    ('z_rate', np.float64),
])

# SBET structured dtype (standard format)
LGF_SBET_DTYPE = np.dtype([
    ('time',   np.float64),
    ('lgf_x',  np.float64),
    ('lgf_y',  np.float64),
    ('lgf_z',  np.float64),
    ('x_vel',  np.float64),
    ('y_vel',  np.float64),
    ('z_vel',  np.float64),
    ('roll',   np.float64),
    ('pitch',  np.float64),
    ('heading',np.float64),
    ('wander', np.float64),
    ('x_accel',np.float64),
    ('y_accel',np.float64),
    ('z_accel',np.float64),
    ('x_rate', np.float64),
    ('y_rate', np.float64),
    ('z_rate', np.float64),
])

NUM_LASERS        = 16
BLOCK_FLAG        = 0xEEFF  # little-endian 0xFFEE
VELODYNE_PORT     = 2368
PACKET_SIZE       = 1206
NUM_BLOCKS        = 12
CHANNELS_PER_BLOCK = 32     # 2 firing sequences × 16 lasers
# VLP-16 elevation angles indexed by laser_id
LASER_ANGLES_VLP16 = [
    -15, 1, -13, 3, -11, 5, -9, 7,
    -7,  9, -5, 11, -3, 13, -1, 15
]
# VLP-16 optical center correction indexed by laser_id
VERTICAL_CORRECTION_VLP16 = [
    11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1,
     5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2
]

def pcap_to_numpy(filepath: str, timing_offsets, return_type_table, min_range=1):
    """
    Read all Velodyne VLP-16 packets from a PCAP file and return two
    numpy arrays. One for the lidar data and another for the sweeping angle.

    Columns for the lidar data:
        0  intensity             [0–255]
        1  laser_id              [0–15]
        2  distance              raw 2 mm units
        3  timestamp_us          microseconds since the top of the hour
        4  time_offset_nanos     beam firing offset in nano seconds
        5  laser_elevation_angle degrees (signed)
        6  beam_return_type      0 for strongest, 1 for last
    """
    lidar_chunks = []
    sweeping_angle_chunks = []

    with open(filepath, 'rb') as f:
        pcap = dpkt.pcap.Reader(f)

        for pcap_ts, buf in pcap:
            try:
                eth = dpkt.ethernet.Ethernet(buf)
                if not isinstance(eth.data, dpkt.ip.IP):
                    continue
                ip = eth.data
                if not isinstance(ip.data, dpkt.udp.UDP):
                    continue
                udp = ip.data
                if udp.dport != VELODYNE_PORT:
                    continue

                pcap_ts_us = int(pcap_ts * 1e6)
                parsed_result = parse_packet(bytes(udp.data), pcap_ts_us, timing_offsets, return_type_table, min_range)

                if parsed_result is not None:
                    lidar_array, sweeping_angle_array = parsed_result
                    lidar_chunks.append(lidar_array)
                    sweeping_angle_chunks.append(sweeping_angle_array)

            except Exception:
                continue

    if not lidar_chunks:
        return np.empty((0, 6), dtype=np.int64), np.empty((0, 3), dtype=np.int64)

    full_lidar_array = np.vstack(lidar_chunks)
    full_sweeping_angle_array = np.vstack(sweeping_angle_chunks)

    return full_lidar_array, full_sweeping_angle_array

def make_firing_time_offsets_table(dual_mode):
    timing_offsets = [[0 for x in range(12)] for y in range(32)]
    full_firing_cycle = 55296  # nano seconds
    single_firing = 2304  # nano seconds
    for x in range(12):
        for y in range(32):
            if dual_mode:
                dataBlockIndex = (x - (x % 2)) + int(y / 16)
            else:
                dataBlockIndex = (x * 2) + int(y / 16)
            dataPointIndex = y % 16
            timing_offsets[y][x] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex)
    return np.array(timing_offsets, dtype=np.uint32)

def make_last_return_type_table():
    last = np.ones((32, 1), dtype=np.uint8)
    return last

def make_strongest_return_type_table():
    strongest = np.zeros((32, 1), dtype=np.uint8)
    return strongest

def make_dual_return_type_table():
    strongest = np.zeros((32, 1), dtype=np.uint8)
    last = np.ones((32, 1), dtype=np.uint8)
    dual_block_return_type = np.hstack((strongest, last))

    return np.tile(dual_block_return_type, (1, 6))

def parse_packet(payload: bytes, pcap_ts_us: int, timing_offsets_table, return_type_table, min_range=1):
    """
    Parse a single 1206-byte Velodyne UDP payload and return 2 arrays, one for lidar data and one for sweeping angle

    Returns an (N, 7) array with columns:
        intensity | laser_id | azimuth | distance | timestamp_us | time_offset_nanos | elevation_angle
    where:
        intensity           - [0-255]
        laser_id            - [0-15]
        distance            – raw 2 mm units          (distance_raw straight from packet)
        timestamp_us        – packet timestamp in microseconds
        time_offset_nanos   – beam firing offset in nanoseconds
        elevation           – degrees (signed integer, from lookup table)
        beam_return_type    - 0 for strongest, 1 for last

    Returns an (N, 3) array with columns:
        azimuth             – hundredths of a degree  [0–35999]
        timestamp_us        – packet timestamp in microseconds
        time_offset_nanos   – sweeping angle time offset in nanoseconds
    """
    min_range_mask = int(min_range*500) # granularity of 2mm

    if len(payload) != PACKET_SIZE:
        return None

    # Last 6 bytes: 4-byte timestamp (µs since the hour) + 2 factory bytes
    timestamp_us = struct.unpack_from('<I', payload, 1200)[0]
    return_type =  struct.unpack_from('<B', payload, 1204)[0]
    product_ID = struct.unpack_from('<B', payload, 1205)[0]

    sweeping_angle_data = []
    rows_beam_data = []
    offset = 0

    for block_idx in range(NUM_BLOCKS):
        flag, azimuth_raw = struct.unpack_from('<HH', payload, offset)
        if flag != BLOCK_FLAG:
            offset += 100
            continue

        sweeping_angle_time_offset = timing_offsets_table[0][block_idx]
        if len(sweeping_angle_data) == 0:
            sweeping_angle_data.append((azimuth_raw, timestamp_us, sweeping_angle_time_offset))
        elif azimuth_raw != sweeping_angle_data[-1][0]: # dual return mode repeats angle
            sweeping_angle_data.append((azimuth_raw, timestamp_us, sweeping_angle_time_offset))

        offset += 4  # skip flag + azimuth field

        for channel in range(CHANNELS_PER_BLOCK):
            distance_raw, intensity = struct.unpack_from('<HB', payload, offset)
            offset += 3

            laser_id  = channel % NUM_LASERS
            elevation = LASER_ANGLES_VLP16[laser_id]
            time_offset_nanos = timing_offsets_table[channel][block_idx]
            beam_return_type = return_type_table[channel][block_idx]

            rows_beam_data.append((
                intensity,
                laser_id,
                distance_raw,     # 2 mm units, straight from packet
                timestamp_us,
                time_offset_nanos,
                elevation,
                beam_return_type,
            ))

    if not rows_beam_data:
        return None

    lidar_array = np.array(rows_beam_data, dtype=np.int64)
    filtered_lidar_array = lidar_array[lidar_array[:, 2] > min_range_mask] # keep only beams with distance greater than min_range_mask
    sweeping_angle_array = np.array(sweeping_angle_data, dtype=np.int64)

    return filtered_lidar_array, sweeping_angle_array

def read_sbet_numpy(path: Path) -> np.ndarray:
    """
    Read an SBET file into a NumPy array.

    Returns
    -------
    np.ndarray
    """
    size = path.stat().st_size
    num_floats = int(size/8)
    num_points = int(num_floats/17)
    return np.memmap(path, dtype='float64', mode='r', shape=(num_points, 17))

def read_sbet_numpy_structured(path: Path) -> np.ndarray:
    """
    Read an SBET file into a structured NumPy array.

    Returns
    -------
    np.ndarray of dtype=SBET_DTYPE
    """
    size = path.stat().st_size
    num_floats = int(size/8)
    num_points = int(num_floats/17)
    data = np.memmap(path, dtype='float64', mode='r', shape=(num_points, 17))
    return data.view(dtype=SBET_DTYPE)

def load_lidar_mat_file(path: Path) -> np.ndarray:
    # Load the .mat
    # Make sure there is only one variable (lidar data matrix) in the mat file
    mat_contents = sio.loadmat(path)
    mat_keys = list(mat_contents.keys())
    return mat_contents[mat_keys[3]]

def create_lidar_double_index(num_points, modulus=1e6) -> np.ndarray:
    index = np.arange(1, num_points + 1, dtype=np.uint64)
    id_mod = np.divmod(index, modulus, dtype=np.uint64)
    return np.concatenate((index, id_mod), axis=1)

def double_to_single_index(first_index, second_index, modulus=1e6) -> np.ndarray:
    return first_index*modulus + second_index

def match_timestamps(microseconds: np.ndarray, offset_nanos: np.ndarray, lidar_scan_hour_of_the_day, gps_day_of_week):
    nanos = microseconds*1000 + offset_nanos
    seconds = nanos*1e-9
    seconds_of_the_day = seconds + lidar_scan_hour_of_the_day * 3600
    return seconds_of_the_day + 86400 * gps_day_of_week

def match_sweeping_angle(sweeping_angle_time_series: np.ndarray, lidar_timestamps) -> np.ndarray:
    return np.zeros_like(lidar_timestamps)

def match_lidar_sbet(sbet_lgf: np.ndarray, lidar_timestamps) -> np.ndarray:
    sbet_structured = sbet_lgf.view(dtype=LGF_SBET_DTYPE)
    sbet_timestamps = sbet_structured['time'].flatten()
    interp_lgf_x    = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['lgf_x'].flatten())
    interp_lgf_y    = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['lgf_y'].flatten())
    interp_lgf_z    = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['lgf_z'].flatten())
    interp_roll    = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['roll'].flatten())
    interp_pitch   = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['pitch'].flatten())
    interp_heading = np.interp(lidar_timestamps, sbet_timestamps, sbet_structured['heading'].flatten())
    return np.concatenate((interp_lgf_x, interp_lgf_y, interp_lgf_z, interp_roll, interp_pitch, interp_heading), axis=1)
    

def geo_to_ecef(points_geo: np.ndarray, a = 6378137.0, e2 = 0.0066943799901414) -> np.ndarray:
    xyz_ecef = np.zeros_like(points_geo)

    lat_rad = points_geo[:, 0]
    lon_rad = points_geo[:, 1]
    h       = points_geo[:, 2]

    slat = np.sin(lat_rad)
    clat = np.cos(lat_rad)

    slon = np.sin(lon_rad)
    clon = np.cos(lon_rad)

    # Prime-vertical radius of curvature or transverse radius of curvature
    # usually denoted N (wikipedia), but in this code it is noted r
    r = a / (np.sqrt(1 - e2 * slat * slat))
    
    # Cartesian coordinates
    xyz_ecef[:, 0] = (r + h) * clat * clon
    xyz_ecef[:, 1] = (r + h) * clat * slon
    xyz_ecef[:, 2] = (r * (1 - e2) + h) * slat

    return xyz_ecef

def calculate_enu_matrix(lat_rad, lon_rad) -> np.ndarray:
    slat = np.sin(lat_rad)
    clat = np.cos(lat_rad)
  
    slon = np.sin(lon_rad)
    clon = np.cos(lon_rad)
    
    return np.array([[-slon, clon, 0], [-slat*clon, -slat*slon, clat], [clat*clon, clat*slon, slat]])
    
def get_conversion_struct(lat_rad, lon_rad, h, calculate_lgf_matrix):
    conversion_struct = {}
    conversion_struct['latitude'] = lat_rad
    conversion_struct['longitude'] = lon_rad
    conversion_struct['ellipsoid_height'] = h
    
    conversion_struct['llh_0'] = np.array([[lat_rad, lon_rad, h]])
    conversion_struct['lgf_origin_in_ECEF'] = geo_to_ecef(conversion_struct['llh_0'])
    conversion_struct['ecef2lgf'] = calculate_lgf_matrix(lat_rad, lon_rad)
    
    return conversion_struct

def transform_lgf_to_ecef(lgf_xyz: np.ndarray, conversion_struct) -> np.ndarray:
    points = conversion_struct['ecef2lgf'].T @ lgf_xyz.T
    return points.T + conversion_struct['lgf_origin_in_ECEF']

def transform_sbet_to_lgf(sbet: np.ndarray, conversion_struct) -> np.ndarray:
    xyz_shift = geo_to_ecef(sbet[:,1:4]) - conversion_struct['lgf_origin_in_ECEF']
    points_lgf = conversion_struct['ecef2lgf'] @ xyz_shift.T
    return np.concatenate((sbet[:, 0:1], points_lgf.T, sbet[:,4:]), axis=1)

def vlp16_spherical_to_cartesian(sweeping_angle: np.ndarray, elevation_angle: np.ndarray, range: np.ndarray):
    saz = np.sin(sweeping_angle)
    caz = np.cos(sweeping_angle)

    sel = np.sin(elevation_angle)
    cel = np.sin(elevation_angle)

    x = range * cel * saz
    y = range * cel * caz
    z = range * sel

    return np.concatenate(x, y, z, axis=1)

def enu_dcm(roll, pitch, heading):
      sr = np.sin(roll)
      cr = np.cos(roll)
      
      sp = np.sin(pitch)
      cp = np.cos(pitch)
      
      sh = np.sin(heading)
      ch = np.cos(heading)
      
      return np.array([[ch*cr+sr*sp*sh, cp*sh, sr*ch-cr*sp*sh], \
                       [sr*sp*ch-cr*sh, cp*ch, -sr*sh-cr*sp*ch], \
                       [-sr*cp, sp, cr*cp]])

def georef_unvectorized(lidar_data: np.ndarray, interpolated_sbet: np.ndarray, lever_arm: np.ndarray, boresight_matrix: np.ndarray) -> np.ndarray:
    # validate input sizes
    num_points = lidar_data.shape[0]
    
    georef_xyz = np.zeros((num_points, 3))
    
    for index in range(num_points):
        
        # platform orientation
        roll = interpolated_sbet[index, 3]
        pitch = interpolated_sbet[index, 4]
        heading = interpolated_sbet[index, 5]
        platform_orientation_matrix = enu_dcm(roll, pitch, heading)
        
        # platform position
        lgf_x = interpolated_sbet[index, 0]
        lgf_y = interpolated_sbet[index, 1]
        lgf_z = interpolated_sbet[index, 2]
        platform_position = np.array([[lgf_x], [lgf_y], [lgf_z]])
        
        # lidar vector in LGF
        lidar_x = lidar_data[index, 0]
        lidar_y = lidar_data[index, 1]
        lidar_z = lidar_data[index, 2]
        lidar_vector_vlp16 = np.array([[lidar_x], [lidar_y], [lidar_z]])
        lidar_vector_ins = boresight_matrix @ lidar_vector_vlp16
        lidar_vector_lgf = platform_orientation_matrix @ lidar_vector_ins
        
        # optical center in LGF
        optical_center_lgf = platform_orientation_matrix @ lever_arm
        
        # georeferencing
        lidar_vector_georef = platform_position + optical_center_lgf + lidar_vector_lgf
        georef_xyz[index] = lidar_vector_georef.T
        
    return georef_xyz

class MyTestCase(unittest.TestCase):

    def test_las_format(self):
        # pcap files
        pcap_folder = '../vol18m_20251128'
        files = list(Path(pcap_folder).glob('*.pcap'))

        # min distance filter
        min_distance_filter = 0

        # dual return lidar processing tables
        dual_return_type_table = make_dual_return_type_table()
        dual_offset_nanos = make_firing_time_offsets_table(True)

        # lidar scan time and day of the week
        lidar_scan_hour_of_the_day = 15
        gps_day_of_week = 2

        # calibrated values
        lever_arm = np.zeros((3, 1))
        lever_arm[0, 0] = -0.0365
        lever_arm[1, 0] = 0.1151
        lever_arm[2, 0] = -0.0321

        boresight_matrix = enu_dcm(np.deg2rad(0.0669), np.deg2rad(-90.4758), np.deg2rad(0.0812))

        # read sbet
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))

        # convert to ENU LGF
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846, calculate_enu_matrix)
        sbet_enu = transform_sbet_to_lgf(sbet, enu_conversion_struct)

        # write to las file
        lidar_tools_header = laspy.LasHeader(point_format=1, version="1.2")
        # Define extra dims when creating a new file
        extra_dims = [
            laspy.ExtraBytesParams(name="Coord.__Z", type=np.float64),
        ]

        lidar_tools_header = laspy.LasHeader(point_format=1, version="1.2")
        lidar_tools_header.add_extra_dims(extra_dims)

        # Scale & offset prevent precision loss when storing floats as int32 internally
        lidar_tools_header.offsets = np.array([299182, 5191620, 0])
        lidar_tools_header.scales = np.array([0.001, 0.001, 0.001])  # 1 mm precision

        # --- 3. Build the LasData object and assign arrays ---
        with laspy.open("output_vol20251125_pcaps_scan_angle_utm19_HT2.las", mode="w", header=lidar_tools_header) as writer:
            for pcap_file in files:
                print(pcap_file)
                print("decoding raw data")
                full_lidar_array, full_sweeping_angle_array = pcap_to_numpy(pcap_file,
                                                                            dual_offset_nanos, dual_return_type_table,
                                                                            min_distance_filter)
                # timestamp matching
                print("timestamp matching")
                nanos = full_lidar_array[:, 3:4] * 1000 + full_lidar_array[:, 4:5]
                lidar_raw_timestamps = nanos * 1e-9
                lidar_timestamps = match_timestamps(full_lidar_array[:, 3:4],
                                                    full_lidar_array[:, 4:5],
                                                    lidar_scan_hour_of_the_day, gps_day_of_week)

                # return number
                return_number = full_lidar_array[:, 6:7]

                # sweeping angle time matching
                print("sweeping angle time matching")
                sweeping_angle_timestamps = match_timestamps(full_sweeping_angle_array[:, 1:2],
                                                             full_sweeping_angle_array[:, 2:3],
                                                             lidar_scan_hour_of_the_day, gps_day_of_week)

                # convert sweeping angle to degrees
                sweeping_angle_degrees = full_sweeping_angle_array[:, 0:1] * 1e-2

                # convert range to meters
                range = full_lidar_array[:, 2] * 0.002  # granularity of 2 mm

                # vertical correction
                laser_vertical_correction = np.array(VERTICAL_CORRECTION_VLP16)
                laser_id = full_lidar_array[:, 1]
                vertical_correction = 0.001 * laser_vertical_correction[laser_id]

                # Convert to Cartesian coordinates
                print("Convert to Cartesian coordinates")
                sweeping_angle_rad = np.deg2rad(sweeping_angle_degrees)
                sweeping_x = np.cos(sweeping_angle_rad)
                sweeping_y = np.sin(sweeping_angle_rad)

                caz = np.interp(lidar_timestamps.flatten(), sweeping_angle_timestamps.flatten(), sweeping_x.flatten())
                saz = np.interp(lidar_timestamps.flatten(), sweeping_angle_timestamps.flatten(), sweeping_y.flatten())

                elevation_degrees = full_lidar_array[:, 5]
                elevation_angle = np.deg2rad(elevation_degrees)
                sel = np.sin(elevation_angle)
                cel = np.cos(elevation_angle)

                x = range * cel * saz
                y = range * cel * caz
                z = range * sel + vertical_correction

                # intensity and return type
                intensity = full_lidar_array[:, 0:1]

                # Convert sweeping angle back to degrees 0-360
                matched_sweeping_angles_degrees = np.rad2deg(np.arctan2(saz, caz))

                # make lidar table
                vlp16_lidar_table = np.concatenate(
                    (x.reshape((-1, 1)), y.reshape((-1, 1)), z.reshape((-1, 1)), intensity, lidar_timestamps,
                     lidar_raw_timestamps, laser_id.reshape((-1, 1)), range.reshape((-1, 1)),
                     matched_sweeping_angles_degrees.reshape((-1, 1)), elevation_degrees.reshape((-1, 1))), axis=1)

                # interpolate sbet data to lidar timestamps
                print("interpolate sbet data to lidar timestamps")
                interpolated_sbet = match_lidar_sbet(sbet_enu, lidar_timestamps)

                # georef
                print("georef")
                georef_xyz = georef_unvectorized(vlp16_lidar_table, interpolated_sbet, lever_arm, boresight_matrix)

                # convert back to ECEF
                print("convert back to ECEF")
                georef_xyz_ecef = transform_lgf_to_ecef(georef_xyz, enu_conversion_struct)

                # convert to UTM19 with HT2_1997
                print("UTM19 with HT2_1997 PROJ pipeline")
                geoid_tif = "../grid/HT2_1997.tif"

                pipeline = f"""
                                                 +proj=pipeline
                                                   +step +proj=cart +inv +ellps=WGS84
                                                   +step +proj=vgridshift +grids={geoid_tif}
                                                   +step +proj=utm +zone=19 +ellps=WGS84
                                                 """

                transformer = Transformer.from_pipeline(pipeline)

                E, N, H = transformer.transform(georef_xyz_ecef[:, 0], georef_xyz_ecef[:, 1], georef_xyz_ecef[:, 2])
                point_data = np.hstack((E.reshape((-1, 1)), N.reshape((-1, 1)), H.reshape((-1, 1))))

                # write to las file
                print("write to las file")
                point_record = laspy.ScaleAwarePointRecord.zeros(point_data.shape[0], header=lidar_tools_header)
                point_record.x = point_data[:, 0]
                point_record.y = point_data[:, 1]
                point_record.z = point_data[:, 2]
                point_record.intensity = full_lidar_array[:, 0]
                point_record.return_number = full_lidar_array[:, 6]
                point_record.user_data = full_lidar_array[:, 1]
                point_record.scan_angle_rank = matched_sweeping_angles_degrees
                point_record.gps_time = vlp16_lidar_table[:, 4]
                writer.write_points(point_record)

    def test_georef_utm19_HT2(self):
        # Load the .mat file
        vlp16_p1 = load_lidar_mat_file(Path("../VLP16/raw_data_18m_P1.mat"))
        intensity = vlp16_p1[:, 3]
        # print(vlp16_p1.shape)

        # read sbet
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))

        # convert to ENU LGF
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846,
                                                      calculate_enu_matrix)
        sbet_enu = transform_sbet_to_lgf(sbet, enu_conversion_struct)

        # interpolate sbet data to lidar timestamps
        lidar_timestamps = vlp16_p1[:, 4:5]
        interpolated_sbet = match_lidar_sbet(sbet_enu, lidar_timestamps)

        lever_arm = np.zeros((3, 1))
        lever_arm[0, 0] = -0.0365
        lever_arm[1, 0] = 0.1151
        lever_arm[2, 0] = -0.0321

        boresight_matrix = enu_dcm(np.deg2rad(0.0669), np.deg2rad(-90.4758), np.deg2rad(0.0812))

        # georef
        georef_xyz = georef_unvectorized(vlp16_p1, interpolated_sbet, lever_arm, boresight_matrix)

        # convert back to ECEF
        georef_xyz_ecef = transform_lgf_to_ecef(georef_xyz, enu_conversion_struct)

        # convert to UTM19 with HT2_1997
        geoid_tif = "../grid/HT2_1997.tif"

        pipeline = f"""
                        +proj=pipeline
                          +step +proj=cart +inv +ellps=WGS84
                          +step +proj=vgridshift +grids={geoid_tif}
                          +step +proj=utm +zone=19 +ellps=WGS84
                        """

        transformer = Transformer.from_pipeline(pipeline)

        E, N, H = transformer.transform(georef_xyz_ecef[:,0], georef_xyz_ecef[:,1], georef_xyz_ecef[:,2])

        # write to las file


        header = laspy.LasHeader(point_format=2, version="1.4")

        # Scale & offset prevent precision loss when storing floats as int32 internally
        header.offsets = np.array([E.min(), N.min(), H.min()])
        header.scales = np.array([0.001, 0.001, 0.001])  # 1 mm precision

        # --- 3. Build the LasData object and assign arrays ---
        las = laspy.LasData(header=header)

        las.x = E
        las.y = N
        las.z = H
        las.intensity = intensity

        # --- 5. Write to disk ---
        output_path = "output_utm19_HT2.las"
        las.write(output_path)

    def test_cart_to_UTM19_with_grid(self):
        geoid_tif = "../grid/HT2_1997.tif"

        pipeline = f"""
                +proj=pipeline
                  +step +proj=cart +inv +ellps=WGS84
                  +step +proj=vgridshift +grids={geoid_tif}
                  +step +proj=utm +zone=19 +ellps=WGS84
                """

        transformer = Transformer.from_pipeline(pipeline)

        x, y, z = 1266772.3408, -4295099.0467, 4526652.8773  # ECEF coords (meters)
        easting, northing, ortho_h = transformer.transform(x, y, z)

        print(f"Easting: {easting:.3f}, Northing: {northing:.3f}, Height: {ortho_h:.3f}")

    def test_geo_to_cart(self):
        transformer = Transformer.from_crs("epsg:4979", "epsg:4978")  # Lon/Lat/Elev to ECEF
        lon, lat, elev = -73.5674, 45.5019, 50.0  # Example: Montreal, QC (degrees, meters)
        x, y, z = transformer.transform(lat, lon, elev)
        print(x)
        self.assertAlmostEqual(x, 1266772.341, delta=0.001)

        print(y)
        self.assertAlmostEqual(y, -4295099.047, delta=0.001)

        print(z)
        self.assertAlmostEqual(z, 4526652.877, delta=0.001)

    def test_georef(self):
        # Load the .mat file
        vlp16_p1 = load_lidar_mat_file(Path("../VLP16/raw_data_18m_P1.mat"))
        # print(vlp16_p1.shape)
        
        # read sbet
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))
        
        # convert to ENU LGF
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846, calculate_enu_matrix)
        sbet_enu = transform_sbet_to_lgf(sbet, enu_conversion_struct)
                
        # interpolate sbet data to lidar timestamps
        lidar_timestamps = vlp16_p1[:, 4:5]
        interpolated_sbet = match_lidar_sbet(sbet_enu, lidar_timestamps)
        
        lever_arm = np.zeros((3,1))
        lever_arm[0, 0] = -0.0365
        lever_arm[1, 0] = 0.1151
        lever_arm[2, 0] = -0.0321

        boresight_matrix = enu_dcm(np.deg2rad(0.0669), np.deg2rad(-90.4758), np.deg2rad(0.0812))
        
        # georef
        georef_xyz = georef_unvectorized(vlp16_p1, interpolated_sbet, lever_arm, boresight_matrix)

        # write to txt file
        np.savetxt('output.txt', georef_xyz)

        # write to las file

        # extract coordinate vectors:
        x = georef_xyz[:, 0]
        y = georef_xyz[:, 1]
        z = georef_xyz[:, 2]

        
        header = laspy.LasHeader(point_format=2, version="1.4")
        
        # Scale & offset prevent precision loss when storing floats as int32 internally
        #header.offsets = np.array([x.min(), y.min(), z.min()])
        header.scales  = np.array([0.001, 0.001, 0.001])   # 1 mm precision

        # --- 3. Build the LasData object and assign arrays ---
        las = laspy.LasData(header=header)

        las.x = x
        las.y = y
        las.z = z

        # --- 5. Write to disk ---
        output_path = "output.las"
        las.write(output_path)
        
        
    
    def test_match_lidar_sbet(self):
        # Load the .mat file
        vlp16_p1 = load_lidar_mat_file(Path("../VLP16/raw_data_18m_P1.mat"))
        # print(vlp16_p1.shape)
        
        # read sbet
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))
        
        # convert to ENU LGF
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846, calculate_enu_matrix)
        sbet_enu = transform_sbet_to_lgf(sbet, enu_conversion_struct)
                
        # interpolate sbet data to lidar timestamps
        lidar_timestamps = vlp16_p1[0:2, 4:5]
        interpolated_data = match_lidar_sbet(sbet_enu, lidar_timestamps)
        
        # print(interpolated_data.shape)
        
        expected_interpolated_data = np.array([[-2.31136246611065, -4.55665351074313, 16.3323607054406, -0.0163366887555277, -0.0417006708371731, 5.51024342673119], \
                                               [-2.3112863561399, -4.55673011607167, 16.3323599274978, -0.0163436210336388, -0.0417041231999259, 5.51024448807145]])
        npt.assert_allclose(interpolated_data, expected_interpolated_data, rtol=1e-8)
    
    def test_load_lidar_mat_file(self):
        # Load the .mat file
        vlp16_p1 = load_lidar_mat_file(Path("../VLP16/raw_data_18m_P1.mat"))
        # print(vlp16_p1.shape)
        
        # read sbet
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))
        
        # convert to ENU LGF
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846, calculate_enu_matrix)
        sbet_enu = transform_sbet_to_lgf(sbet, enu_conversion_struct)
        
        # example of boolean indexing that won't work
        sbet_gps_time_bad_for_indexing = sbet_enu[:, 0:1]
        lidar_gps_time_bad_for_indexing = vlp16_p1[:, 4:5]
        
        I_vlp_after_sbet_start = lidar_gps_time_bad_for_indexing > np.min(sbet_gps_time_bad_for_indexing)
        I_vlp_before_sbet_end = lidar_gps_time_bad_for_indexing < np.max(sbet_gps_time_bad_for_indexing)
        I_valid_lidar_bad_indexing = I_vlp_after_sbet_start & I_vlp_before_sbet_end
        print(I_valid_lidar_bad_indexing.shape)
        print("I_valid_lidar_bad_indexing.shape = (1005, 1): this dimension shape won't allow to select the rest of the matrix")
        
        # example of boolean indexing that will work
        sbet_gps_time_good_for_indexing = sbet_enu[:, 0]
        lidar_gps_time_good_for_indexing = vlp16_p1[:, 4]
        
        I_vlp_after_sbet_start = lidar_gps_time_good_for_indexing > np.min(sbet_gps_time_good_for_indexing)
        I_vlp_before_sbet_end = lidar_gps_time_good_for_indexing < np.max(sbet_gps_time_good_for_indexing)
        I_valid_lidar = I_vlp_after_sbet_start & I_vlp_before_sbet_end
        print(I_valid_lidar.shape)
        print("I_valid_lidar.shape = (1005,): this dimension shape allows to select the rest of the matrix")
        
        valid_lidar = vlp16_p1[I_valid_lidar]
        print(valid_lidar.shape)
    
    def test_transform_sbet_to_lgf(self):
        sbet = read_sbet_numpy(Path("../SBET/SBET_CALIB_VOL_18M.out"))
        
        enu_conversion_struct = get_conversion_struct(0.817648708223942, -1.25025230582025, 128.846, calculate_enu_matrix)
        
        sub_sample_sbet = sbet[0:2, :]
        
        expected_sbet_enu = np.array([[229048.006000042, 11.1106852133820, -3.97029293197942, 34.2234582315404, -6.82953271673068, -12.1806020619000, -0.388240796572005, 0.0209191309445385, -0.563417497268485, 2.07914486803658, 0, 2.02718722698043, -0.373118363374243, -3.63371879266057, 0.0467727407672790, 0.273599111291664, 0.151239372210387], \
                                      [229048.011000156, 11.1701391049565, -4.00598131333360, 34.2215502926780, -7.10943860140797, -12.1051457784417, -0.411117080099086, 0.0192731805039448, -0.565025310537323, 2.10030061968717, 0, 4.91682721747373, 0.448052629448701, -4.20067636507991, -0.0363052749389747, 0.402753666005882, 0.0692091630865624]])
            
        sbet_enu = transform_sbet_to_lgf(sub_sample_sbet, enu_conversion_struct)
        
        # 1e-4 meter tolerance for position
        npt.assert_allclose(sbet_enu[:, 1:4], expected_sbet_enu[:, 1:4], rtol=1e-4)
        
        # 1e-12 tolerance for time and other columns of sbet
        npt.assert_allclose(sbet_enu[:, 0:1], expected_sbet_enu[:, 0:1], rtol=1e-12)
        npt.assert_allclose(sbet_enu[:, 4:], expected_sbet_enu[:, 4:], rtol=1e-12)
    
    def test_calculate_enu_matrix(self):
        lat_0 = 0.817648708223942
        lon_0 = -1.25025230582025
        enu_matrix = calculate_enu_matrix(lat_0, lon_0)
        
        expected_enu_matrix = np.array([[0.949064146816691, 0.315082918018584, 0], \
                                        [-0.229865500320633, 0.692379981486419, 0.683938457026077], \
                                        [0.215497324784904, -0.649101468192579, 0.729539708995191]])
        
         
        # Check for near-equality (1e-12 tolerance)
        npt.assert_allclose(enu_matrix, expected_enu_matrix, rtol=1e-12)

    def test_geo_to_ecef(self):
        
        # test single point
        riki = np.array([[48.4390, 68.5350, 0]])
        riki_ecef = geo_to_ecef(riki)
        expected_riki_ecef = np.array([[-1353503.021, 886846.720, -6148769.102]])
        # Check for near-equality (1 mm tolerance)
        npt.assert_allclose(riki_ecef, expected_riki_ecef, rtol=1e-3)
        
        # test vectorisation with 2 points 
        riki = np.array([[48.4390, 68.5350, 0], [48.4390, 68.5350, 1]])
        riki_ecef = geo_to_ecef(riki)
        expected_riki_ecef = np.array([[-1353503.021, 886846.720, -6148769.102], [-1353503.232, 886846.859, -6148770.069]])
        # Check for near-equality (1 mm tolerance)
        npt.assert_allclose(riki_ecef, expected_riki_ecef, rtol=1e-3)


if __name__ == '__main__':
    unittest.main()
