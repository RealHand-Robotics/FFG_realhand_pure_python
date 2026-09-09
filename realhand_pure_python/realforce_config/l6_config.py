FINGER_CONFIGS = {
    'thumb_abduction': {
        'name': 'thumb abduction',
        'joints': [0, 1, 2],
        'weights': {
            'v1': [0, 0, 1],
            'v2': [0, 1, 0]
        },
        'robot_idx': 0,
        'type': 'thumb',
        'reverse_motion': False,
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.3,
            'extended_exp_factor': 10
        }
    },
    'thumb_root_flexion': {
        'name': 'thumb flexion',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [0.2, 0, 0.8],
            'v2': [0.6, 0, 0.4]
        },
        'robot_idx': 1,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 2
        }
    },
    'index_root_flexion': {
        'name': 'index finger',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 3,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 5
        }
    },
    'middle_root_flexion': {
        'name': 'middle finger',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 5,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 5
        }
    },
    'ring_root_flexion': {
        'name': 'ring finger',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 7,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 5
        }
    },
    'pinky_root_flexion': {
        'name': 'little finger',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 9,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    }
}

MAPPING_ORDER = [
    'thumb_abduction', 'thumb_root_flexion',
    'index_root_flexion', 'middle_root_flexion', 'ring_root_flexion', 'pinky_root_flexion'
]

MULTI_SEGMENT_CONFIG = {
    'states': [
        'original',
        # 'opose',
        'fist'
    ],
    'state_names': {
        'original': 'open hand',
        # 'opose': 'O pose',
        'fist': 'fist'
    }
}
MULTI_SEGMENT_CONFIG_FROZEN = tuple(MULTI_SEGMENT_CONFIG['states'])

ROBOT_OPOSE_LEFT = [
    1.4, 0.5, 0.0,
    0.48, 0.0,
    0.48, 0.0,
    0.48, 0.0,
    0.48, 0.0
]

ROBOT_OPOSE_RIGHT = [
    1.4, 0.5, 0.0,
    0.48, 0.0,
    0.48, 0.0,
    0.48, 0.0,
    0.48, 0.0
]

ROBOT_ORIGINAL_LEFT = [
    0.0, 0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0
]

ROBOT_ORIGINAL_RIGHT = [
    0.0, 0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0
]

# Fist pose (using the URDF upper limit)
ROBOT_FIST_LEFT = [
    1.53, 0.73, 0.66,
    1.22, 1.08,
    1.22, 1.08,
    1.22, 1.08,
    1.22, 1.08
]

ROBOT_FIST_RIGHT = [
    1.53, 0.73, 0.66,
    1.22, 1.08,
    1.22, 1.08,
    1.22, 1.08,
    1.22, 1.08
]

# Motor output constraint configuration
# Format: {'min': minimum value, 'max': maximum value, 'enabled': whether enabled}
# None means this motor is unconstrained
MOTOR_CONSTRAINTS = {
    'left': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: thumb flexion
        {'min': 15, 'max': 255, 'enabled': True},   # motor 1: thumb abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: index finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: middle finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: ring finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: little finger
    ],
    'right': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: thumb flexion
        {'min': 18, 'max': 255, 'enabled': True},   # motor 1: thumb abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: index finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: middle finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: ring finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: little finger
    ]
}