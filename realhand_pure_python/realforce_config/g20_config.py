# Finger configuration constants
FINGER_CONFIGS = {
    # Explanation:
    # robot_idx: index in the URDF joint sequence

    # Weight the three thumb-rotation joints (human-hand joints 0/1/2) for URDF joint 1 (index 0).
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
            'extended_exp_factor': 1.0
        }
    },
    # Weight the three thumb-abduction joints (human-hand joints 0/1/2) for URDF joint 2 (index 1).
    'thumb_abduction': {
        'name': 'thumb abduction',
        'joints': [0, 1, 2],
        'weights': {
            'v1': [0.7, 0.3, 0],
            'v2': [0.7, 0.3, 0]
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
            'scale_factor': 1.2,
            'extended_exp_factor': 1.0
        }
    },
    # Weight the three thumb-root-flexion joints (human-hand joints 2/3/4) for URDF joint 3 (index 2).
    'thumb_root_flexion': {
        'name': 'thumb root flexion',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 2,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        # {
        #     'trigger_finger': 'thumb_abduction',
        #     'threshold': 0.3,
        #     'low_weight_config': {
        #         'joints': [2, 3, 4],
        #         'weights': [1, 0, 0],
        #         'reverse_motion': False
        #     },
        #     'high_weight_config': {
        #         'joints': [2, 3, 4],
        #         'weights': [0.3, 0.0, 0.7],
        #         'reverse_motion': False
        #     }
        # },
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 30
        }
    },
    # Weight the three thumb-tip-flexion joints (human-hand joints 2/3/4) for URDF joint 4 (index 3).
    'thumb_end_flexion': {
        'name': 'thumb tip flexion',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 3,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1,
            'extended_exp_factor': 50
        }
    },
    # Weight the index-finger roll (abduction) joint (human-hand joint 5) for URDF joint 4 (index 3).
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
    # Weight index-finger root flexion (human-hand joints 6/7/8) for URDF joint 4 (index 3).
    'index_root_flexion': {
        'name': 'index finger',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
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
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    # Weight index-finger distal flexion (human-hand joints 6/7/8) for URDF joint 4 (index 3).
    'index_end_flexion': {
        'name': 'index finger',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
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
            'scale_factor': 1,
            'extended_exp_factor': 30
        }
    },
    # Weight the middle-finger roll (abduction) joint (human-hand joint 5) for URDF joint 4 (index 3).
    'middle_roll': {
        'name': 'middle finger',
        'joints': [9],
        'weights': {
            'v1': [1],
            'v2': [1]
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
            'extended_exp_factor': 1.0
        }
    },
    # Weight middle-finger root flexion (human-hand joints 10/11/12) for URDF joint 6 (index 5).
    'middle_root_flexion': {
        'name': 'middle finger',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 10,
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
    },
    # Weight middle-finger distal flexion (human-hand joints 10/11/12) for URDF joint 6 (index 5).
    'middle_end_flexion': {
        'name': 'middle finger',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 11,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1,
            'extended_exp_factor': 30
        }
    },
    # Weight the ring-finger roll (abduction) joint (human-hand joint 5) for URDF joint 4 (index 3).
    'ring_roll': {
        'name': 'ring finger',
        'joints': [13],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 13,
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
    # Weight ring-finger root flexion (human-hand joints 14/15/16) for URDF joint 8 (index 7).
    'ring_root_flexion': {
        'name': 'ring finger',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 14,
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
    },
    # Weight ring-finger distal flexion (human-hand joints 14/15/16) for URDF joint 8 (index 7).
    'ring_end_flexion': {
        'name': 'ring finger',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 15,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1,
            'extended_exp_factor': 30
        }
    },
    # Weight the little-finger roll (abduction) joint (human-hand joint 5) for URDF joint 4 (index 3).
    'pinky_roll': {
        'name': 'little finger',
        'joints': [17],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 17,
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
    # Weight little-finger root flexion (human-hand joints 18/19/20) for URDF joint 10 (index 9).
    'pinky_root_flexion': {
        'name': 'little finger',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 18,
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
    },
    # Weight little-finger distal flexion (human-hand joints 18/19/20) for URDF joint 10 (index 9).
    'pinky_end_flexion': {
        'name': 'little finger',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 19,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1,
            'extended_exp_factor': 30
        }
    }
}

# Mapping order
MAPPING_ORDER = [
    'thumb_rotate', 'thumb_abduction', 'thumb_root_flexion', 'thumb_end_flexion',
    'index_roll', 'index_root_flexion', 'index_end_flexion',
    'middle_roll', 'middle_root_flexion', 'middle_end_flexion',
    'ring_roll', 'ring_root_flexion','ring_end_flexion',
    'pinky_roll', 'pinky_root_flexion', 'pinky_end_flexion'
]

# Default three-state configuration
MULTI_SEGMENT_CONFIG = {
    'states': [
        'original',
        'opose',
        # 'fist'  # uncomment to enable three-segment mapping
    ],
    'state_names': {
        'original': 'open hand',
        'opose': 'O pose',
        # 'fist': 'fist'
    }
}
MULTI_SEGMENT_CONFIG_FROZEN = tuple(MULTI_SEGMENT_CONFIG['states'])

ROBOT_ORIGINAL_LEFT = [
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.2, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    -0.2, 0.0, 0.0, 0.0,
    -0.2, 0.0, 0.0, 0.0
]

ROBOT_ORIGINAL_RIGHT = [
    0.0, 0.0, 0.0, 0.0, 0.0,
    -0.2, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.2, 0.0, 0.0, 0.0,
    0.2, 0.0, 0.0, 0.0
]

ROBOT_OPOSE_LEFT = [
    0.6, 1.2, 0.5, 0.6, 0.0,
    0.0, 0.7, 1.08, 0.00,
    0.0, 0.7, 1.08, 0.00 ,
    0.0, 0.7, 1.08, 0.00,
    0.0, 0.7, 1.08, 0.00
]

ROBOT_OPOSE_RIGHT = [
    0.6, 1.2, 0.5, 0.6, 0.0,
    0.0, 0.7, 1.08, 0.00,
    0.0, 0.7, 1.08, 0.00,
    0.0, 0.7, 1.08, 0.00,
    0.0, 0.7, 1.08, 0.00
]

ROBOT_FIST_RIGHT = [
    1.39, 1.57, 0.83, 1.25, 1.29,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55
]

ROBOT_FIST_LEFT = [
    1.39, 1.57, 0.83, 1.25, 1.29,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55,
    0, 1.22, 1.75, 1.55
]

# Motor output constraint configuration (20motor)
MOTOR_CONSTRAINTS = {
    'left': [
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 80, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
    ],
    'right': [
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 80, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
    ]
}
