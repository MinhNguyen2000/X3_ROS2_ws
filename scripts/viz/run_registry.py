"""
Registry of recorded runs, shared by visualize_path.py and visualize_repeats.py.

Keys are run files under SAVED_PATHS_DIR, written without the .json extension:
    '<model_folder>/<path_name>_<world>_<ddmmyy>_<HHMM>'

PATH_LISTS     : mixed runs for policy comparison plots (uncomment what you want to plot).
REPEAT_BATCHES : the repeated runs of ONE policy on ONE sequence, used for median + band plots.
Repeat batches are defined once below and referenced from PATH_LISTS, so a run is never listed twice.
"""
from viz_common import planner_label

# ===== Repeat batches (world_lab, 2026-10-15) =====

_P1_W1_DELTA_FINAL = [
    'TD3_00386_1000/path1_world_1_211026_2202',
    # 'TD3_00387_1000/',
    # 'TD3_00388_1000/',
    # 'TD3_00389_1000/',
    # 'TD3_00390_1000/',
    # 'TD3_00391_1000/',
    # 'TD3_00392_1000/',
    # 'TD3_00393_1000/',
    # 'TD3_00394_1000/',
    # 'TD3_00395_1000/',
]

_P1_DELTA = [
    'TD3_00395_1000/path1_world_lab_151026_2036',
    'TD3_00395_1000/path1_world_lab_151026_2037',
    'TD3_00395_1000/path1_world_lab_151026_2039',
    'TD3_00395_1000/path1_world_lab_151026_2040',
    'TD3_00395_1000/path1_world_lab_151026_2041',
    'TD3_00395_1000/path1_world_lab_151026_2042',
    'TD3_00395_1000/path1_world_lab_151026_2044',
    'TD3_00395_1000/path1_world_lab_151026_2045',
    'TD3_00395_1000/path1_world_lab_151026_2046',
    'TD3_00395_1000/path1_world_lab_151026_2048',
]
_P1_DIRECT = [
    'TD3_00404_500/path1_world_lab_151026_2050',
    'TD3_00404_500/path1_world_lab_151026_2052',
    'TD3_00404_500/path1_world_lab_151026_2054',
    'TD3_00404_500/path1_world_lab_151026_2055',
    'TD3_00404_500/path1_world_lab_151026_2057',
    'TD3_00404_500/path1_world_lab_151026_2058',
    'TD3_00404_500/path1_world_lab_151026_2100',
    'TD3_00404_500/path1_world_lab_151026_2102',
    'TD3_00404_500/path1_world_lab_151026_2110',
    'TD3_00404_500/path1_world_lab_151026_2111',
]
_P2_DELTA = [
    'TD3_00388_1000/path2_world_lab_151026_2118',
    'TD3_00388_1000/path2_world_lab_151026_2119',
    'TD3_00388_1000/path2_world_lab_151026_2120',
    'TD3_00388_1000/path2_world_lab_151026_2121',
    'TD3_00388_1000/path2_world_lab_151026_2122',
    'TD3_00388_1000/path2_world_lab_151026_2124',
    'TD3_00388_1000/path2_world_lab_151026_2125',
    'TD3_00388_1000/path2_world_lab_151026_2128',
    'TD3_00388_1000/path2_world_lab_151026_2129',
    'TD3_00388_1000/path2_world_lab_151026_2131',
]
_P2_DIRECT = [
    'TD3_00405_1000/path2_world_lab_151026_2144',
    'TD3_00405_1000/path2_world_lab_151026_2145',
    'TD3_00405_1000/path2_world_lab_151026_2146',
    'TD3_00405_1000/path2_world_lab_151026_2147',
    'TD3_00405_1000/path2_world_lab_151026_2148',
    'TD3_00405_1000/path2_world_lab_151026_2149',
    'TD3_00405_1000/path2_world_lab_151026_2154',
    'TD3_00405_1000/path2_world_lab_151026_2156',
    'TD3_00405_1000/path2_world_lab_151026_2157',
    'TD3_00405_1000/path2_world_lab_151026_2158',
]

# (path, world, planner_label) -> list of run keys
REPEAT_BATCHES = {
    ('path_1', 'world_lab', 'TD3_delta'):  _P1_DELTA,
    ('path_1', 'world_lab', 'TD3_direct'): _P1_DIRECT,
    ('path_2', 'world_lab', 'TD3_delta'):  _P2_DELTA,
    ('path_2', 'world_lab', 'TD3_direct'): _P2_DIRECT,
}

# ===== Policy comparison lists =====
PATH_LISTS = {
    # ===== WORLD 1 =====
    ('path_1', 'world_1'): [
        'APF/path1_world_1_300726_0018',
        # 'TD3_00380_1000/path1_world_1_300726_1029',
        # 'TD3_00384_500/path1_world_1_300726_0031',
        # 'TD3_00384_1000/path1_world_1_300726_1101',
        # 'TD3_00385_1000/path1_world_1_260826_2308'

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_1_300826_1152',
        # 'TD3_00387_1000/path1_world_1_300826_1218',
        # 'TD3_00388_1000/path1_world_1_300826_1246',
        # 'TD3_00389_1000/path1_world_1_300826_1328',
        # 'TD3_00390_1000/path1_world_1_300826_1336',

        'TD3_00400_700/path1_world_1_110926_1830',
        'TD3_00390_800/path1_world_1_110926_1820',
    ],
    ('path_2', 'world_1'): [
        'APF/path2_world_1_300726_1054',
        # 'TD3_00380_1000/path2_world_1_300726_1033',
        # 'TD3_00384_500/path2_world_1_300726_1025',
        # 'TD3_00384_1000/path2_world_1_300726_1107',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path2_world_1_300826_1201',
        # 'TD3_00387_1000/path2_world_1_300826_1222',
        # 'TD3_00388_1000/path2_world_1_300826_1247',
        # 'TD3_00389_1000/path2_world_1_300826_1329',
        # 'TD3_00390_1000/path2_world_1_300826_1338',

        'TD3_00400_700/path2_world_1_110926_1832',
        'TD3_00390_800/path2_world_1_110926_1822',
    ],
    ('path_3', 'world_1'): [
        'APF/path3_world_1_300726_1055',
        # 'TD3_00380_1000/path3_world_1_300726_1035',
        # 'TD3_00384_500/path3_world_1_300726_1050',
        # 'TD3_00384_1000/path3_world_1_300726_1108',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path3_world_1_300826_1202',
        # 'TD3_00387_1000/path3_world_1_300826_1226',
        # 'TD3_00388_1000/path3_world_1_300826_1253',
        # 'TD3_00389_1000/path3_world_1_300826_1329',
        # 'TD3_00390_1000/path3_world_1_300826_1339',

        'TD3_00400_700/path3_world_1_110926_1836',
        'TD3_00390_800/path3_world_1_110926_1824',
    ],
    ('path_4', 'world_1'): [
        'APF/path4_world_1_300726_1056',
        # 'TD3_00380_1000/path4_world_1_300726_1039',
        # 'TD3_00384_500/path4_world_1_300726_1051',
        # 'TD3_00384_1000/path4_world_1_300726_1111',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path4_world_1_300826_1203',
        # 'TD3_00387_1000/path4_world_1_300826_1228',
        # 'TD3_00388_1000/path4_world_1_300826_1257',
        # 'TD3_00389_1000/path4_world_1_300826_1330',
        # 'TD3_00390_1000/path4_world_1_300826_1340',

        'TD3_00400_700/path4_world_1_110926_1837',
        'TD3_00390_800/path4_world_1_110926_1824',
    ],
    # ===== WORLD 2 =====
    ('path_1', 'world_2'): [
        'APF/path1_world_2_130826_1201',
        # 'TD3_00380_1000/path1_world_2_120826_1655',
        # 'TD3_00384_1000/path1_world_2_120826_1650',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_2_300826_1204',
        # 'TD3_00387_1000/path1_world_2_300826_1229',
        # 'TD3_00388_1000/path1_world_2_300826_1323',
        # 'TD3_00389_1000/path1_world_2_300826_1332',
        # 'TD3_00390_1000/path1_world_2_300826_1341',

        'TD3_00400_700/path1_world_2_110926_1840',
        'TD3_00390_800/path1_world_2_110926_1842',
    ],
    # ===== WORLD 3 =====
    ('path_1', 'world_3'): [
        'APF/path1_world_3_130826_1204',
        # 'TD3_00380_1000/path1_world_3_120826_1659',
        # 'TD3_00384_1000/path1_world_3_120826_1700',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_3_300826_1207',
        # 'TD3_00387_1000/path1_world_3_300826_1230',
        # 'TD3_00388_1000/path1_world_3_300826_1324',
        # 'TD3_00389_1000/path1_world_3_300826_1333',
        # 'TD3_00390_1000/path1_world_3_300826_1342',

        'TD3_00400_700/path1_world_3_110926_1849',
        'TD3_00390_800/path1_world_3_110926_1845',
    ],
    # ===== WORLD 4 =====
    ('path_1', 'world_4'): [
        'APF/path1_world_4_130826_1209',
        # 'TD3_00380_1000/path1_world_4_130826_1141',
        # 'TD3_00384_1000/path1_world_4_130826_1137',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_4_300826_1210',
        # 'TD3_00387_1000/path1_world_4_300826_1233',
        # 'TD3_00388_1000/path1_world_4_300826_1326',
        # 'TD3_00389_1000/path1_world_4_300826_1334',
        # 'TD3_00390_1000/path1_world_4_300826_1344',
    ],
    # ===== WORLD 5 =====
    ('path_1', 'world_5'): [
        'APF/path1_world_5_130826_1211',
        # 'TD3_00380_1000/path1_world_5_130826_1215',
        # 'TD3_00384_1000/path1_world_5_130826_1155',

        # # TD3_delta (5 seeds)
        # 'TD3_00386_1000/path1_world_5_300826_1213',
        # 'TD3_00387_1000/path1_world_5_300826_1243',
        # 'TD3_00388_1000/path1_world_5_300826_1327',
        # 'TD3_00389_1000/path1_world_5_300826_1334',
        # 'TD3_00390_1000/path1_world_5_300826_1345',

        'TD3_00400_700/path1_world_5_110926_1850',
        'TD3_00390_800/path1_world_5_110926_1846',
    ],

    # ===== WORLD LAB =====
    ('path_1', 'world_lab'): [
        # # vyaw = 0.3
        # # 'TD3_00386_1000/path1_world_lab_131026_2242',
        # # path1_world_lab_131026_2306',
        # 'TD3_00395_1000/path1_world_lab_131026_2311',
        # # 'TD3_00395_900/path1_world_lab_131026_2312',
        # 'TD3_00404_500/path1_world_lab_141026_0951',

        # # vyaw = 0.4
        # # 'TD3_00386_1000/path1_world_lab_131026_2242',
        # # 'TD3_00386_1000/path1_world_lab_131026_2314',
        # # 'TD3_00387_500/path1_world_lab_141026_0806',
        # # 'TD3_00394_1000/path1_world_lab_141026_0913',
        # 'TD3_00395_1000/path1_world_lab_141026_0916',
        # # 'TD3_00395_900/path1_world_lab_141026_0918',

        # 'TD3_00400_700/path1_world_lab_141026_1621',
        # # 'TD3_00404_500/path1_world_lab_141026_1719',

        # Batch of 10 TD3_delta runs (2026-10-15)
        *_P1_DELTA,

        # Batch of 10 TD3_direct runs (2026-10-15)
        *_P1_DIRECT,
    ],

    ('path_2', 'world_lab'): [
        # # vyaw = 0.3
        # # 'TD3_00386_1000/path2_world_lab_151026_1149',
        # 'TD3_00388_1000/path2_world_lab_151026_1214',
        # # 'TD3_00400_1000/path2_world_lab_151026_1506',
        # 'TD3_00405_1000/path2_world_lab_151026_1527',

        # # # vyaw = 0.4
        # # 'TD3_00387_500/path2_world_lab_151026_1426',
        # # 'TD3_00389_1000/path2_world_lab_151026_1431',
        # # 'TD3_00390_800/path2_world_lab_151026_1434',
        # 'TD3_00393_1000/path2_world_lab_151026_1440',
        # # 'TD3_00394_1000/path2_world_lab_151026_1443',
        # # 'TD3_00395_1000/path2_world_lab_151026_1446',
        # 'TD3_00400_1000/path2_world_lab_151026_1543',

        # Batch of 10 TD3_delta runs (2026-10-15)
        *_P2_DELTA,

        # Batch of 10 TD3_direct runs (2026-10-15)
        *_P2_DIRECT,
    ],
}


# ===== Accessors =====
def get_path_list(path: str, world: str) -> list:
    """Runs to plot together for a policy comparison figure."""
    return PATH_LISTS.get((path, world), [])


def get_repeat_batch(path: str, world: str, policy: str, n_expected: int = 10) -> list:
    """
    Repeated runs of one policy on one sequence. Validates the batch so a stray or
    mislabelled run cannot slip into a median/band figure.
    """
    key = (path, world, policy)
    if key not in REPEAT_BATCHES:
        raise KeyError(f'No repeat batch for {key}. Available: {list(REPEAT_BATCHES)}')

    runs = REPEAT_BATCHES[key]
    if len(runs) != n_expected:
        raise ValueError(f'{key}: expected {n_expected} runs, found {len(runs)}')
    if len(set(runs)) != len(runs):
        raise ValueError(f'{key}: duplicate run keys')

    for r in runs:
        label = planner_label(r.split('/')[0])
        if label != policy:
            raise ValueError(f'{r} is classified as {label}, not {policy}')
        if f'{path.replace("_", "")}_{world}_' not in r:
            raise ValueError(f'{r} does not belong to {path}/{world}')
    return runs