# LMPVC Core Module

This package contains the main loop and other core functionalities such as the high level controller and policy bank.

**Normal usage:**
```
ros2 run lmpvc_core voice_control
```

**Test clients for individual modules:**
```
ros2 run lmpvc_core *module*_test
```

## Voice Control & Main

These are the two components making up the main loop of the system. The actual logic is in voice_control.py, while main.py implements the integration with ROS2 on top of it.

### Configuration

Inside core_config.json you can find a block corresponding to Voice Control:

```
"voice_control": {
    "text_only": true,
    "preamble_file": "preamble.txt",
    "manual_review": true
}
```

**Options:**

*text_only:* set to 'true' to disable microphone in favour of command line input

*preamble_file:* a name of a file inside this directory, containing the "template" for code generation

*manual_review:* set to 'true' to manually verify code on the command line before execution, good for safe testing

## Robot API

This is the high level controller portion of the system. It interfaces with the low level controller and object detection through ROS2 and provides high level methods through which they can be used in generated code.

### Configuration

Inside core_config.json you can find a block corresponding to Robot API:

```
"robot_api": {
    "max_speed": 0.1,
    "starting_speed": 0.05,
    "safety_box": {
        "min": [
            0.0,
            -0.3,
            0.121
        ],
        "max": [
            0.6,
            0.3,
            0.75
        ]
    }
}
```

**Options:**

*max_speed:* set the maximum speed (in m/s, implemented using MoveIt) that can be set through the API

*starting_speed:* set the default speed value to use when starting the system (in m/s, starting_speed<=max_speed)

*safety_box:* CURRENTLY NOT USED! set two boxes to define a box shaped area which the end effector can't leave

*eef_rotation:* change the default orientation of the end effector (euler angles: x,y,z)

**Note:**

Default value of eef_rotation is set for a Franka Emika Panda with the gripper facing down.

## Policy Bank

A system for saving and recalling policies consisting of RobotAPI script.

### Adding a policy

To add a policy, first write .src file similar to the provided examples, roughly in this format:

```
*import statements*
# BODY
*policy code*
# HINT
*hint(s) for code generation*
```

Note that hints increase the size of the context provided to the model, so try have keep them as few and small as possible. Then, add your file to *policies/index.json*. That's it, now the code generation should be able to access your policy.

### Configuration

Inside core_config.json you can find a block corresponding to Policy Bank:

```
"policy_bank": {
    "blacklist": [
        "hello",
        "goodbye"
    ],
    "hint_tag": "# HINT",
    "sub_directory": "policies"
}
```

**Options:**

*blacklist:* add names of policies to make the system ignore them without having to fully remove them

*hint_tag:* used to exctrach hints from policy files, probably leave this as is

*sub_directory:* set a subdirectory inside this one to use for saving policies