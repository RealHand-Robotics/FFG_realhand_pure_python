FINGER_CONFIGS = {
    'thumb_rotate': {
        'name': 'thumb rotation',
        'joints': [1, 2],
        'weights': {
            'v1': [1, 0],
            'v2': [1, 0]
        },
        'robot_idx': 0,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 1
        }
    },
    'thumb_abduction': {
        'name': 'thumb abduction',
        'joints': [0, 1, 2],
        'weights': {
            'v1': [0, 0, 1],
            'v2': [0, 1, 0]
        },
        'robot_idx': 1,
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
        'robot_idx': 2,
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
    'index_roll': {
        'name': 'index finger',
        'joints': [5],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 5,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'index_root_flexion': {
        'name': 'index finger',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 6,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 25
        }
    },
    'middle_root_flexion': {
        'name': 'middle finger',
        'joints': [10, 11, 12],
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
            'scale_factor': 1.0,
            'extended_exp_factor': 25
        }
    },
    'ring_roll': {
        'name': 'ring finger',
        'joints': [13],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 12,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'ring_root_flexion': {
        'name': 'ring finger',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 13,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 25
        }
    },
    'pinky_roll': {
        'name': 'little finger',
        'joints': [17],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 16,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'pinky_root_flexion': {
        'name': 'little finger',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 17,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 30
        }
    }
}

MAPPING_ORDER = [
    'thumb_rotate', 'thumb_abduction', 'thumb_root_flexion',
    'index_roll','index_root_flexion',
    'middle_root_flexion',
    'ring_roll', 'ring_root_flexion',
    'pinky_roll', 'pinky_root_flexion',

]

MULTI_SEGMENT_CONFIG = {
    'states': [
        'original',
        'opose',
        # 'fist'
        ],
    'state_names': {
        'original': 'open hand',
        'opose': 'O pose',
        # 'fist': 'fist'
    }
}
MULTI_SEGMENT_CONFIG_FROZEN = tuple(MULTI_SEGMENT_CONFIG['states'])

ROBOT_OPOSE_LEFT = [
    0.13, 1.13, 0.28, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0,
         0.73, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0
]

ROBOT_OPOSE_RIGHT = [
    0.13, 1.13, 0.28, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0,
         0.73, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0,
    0.0, 0.73, 0.0, 0.0
]

ROBOT_ORIGINAL_LEFT = [
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.22, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,
    -0.22, 0.0, 0.0, 0.0,
    -0.22, 0.0, 0.0, 0.0
]

ROBOT_ORIGINAL_RIGHT = [
    0.0, 0.0, 0.0, 0.0, 0.0,
    -0.22, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0,
    0.22, 0.0, 0.0, 0.0,
    0.22, 0.0, 0.0, 0.0
]

ROBOT_FIST_LEFT = [
    1.1339, 1.9189, 0.5146, 0.7152, 0.7763,
    0, 1.3607, 1.8317, 1.8317,
       1.3607, 1.8317, 0.628,
    0, 1.3607, 1.8317, 0.628,
    0, 1.3607, 1.8317, 0.628
]

ROBOT_FIST_RIGHT = [
    1.1339, 1.9189, 0.5146, 0.7152, 0.7763,
    0, 1.3607, 1.8317, 1.8317,
       1.3607, 1.8317, 0.628,
    0, 1.3607, 1.8317, 0.628,
    0, 1.3607, 1.8317, 0.628
]

# Motor output constraint configuration
# Format: {'min': minimum value, 'max': maximum value, 'enabled': whether enabled}
# None means this motor is unconstrained
MOTOR_CONSTRAINTS = {
    'left': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: thumb flexion
        {'min': 0, 'max': 255, 'enabled': False},   # motor 1: thumb abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: index finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: middle finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: ring finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: little finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 6: index-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 7: ring-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 8: little-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 9: thumb rotation
    ],
    'right': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: thumb flexion
        {'min': 0, 'max': 255, 'enabled': False},   # motor 1: thumb abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: index finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: middle finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: ring finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: little finger
        {'min': 0, 'max': 255, 'enabled': False},   # motor 6: index-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 7: ring-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 8: little-finger abduction
        {'min': 0, 'max': 255, 'enabled': False},   # motor 9: thumb rotation
    ]
}