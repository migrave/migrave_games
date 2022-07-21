# migrave_games
Repository of games designed for the MigrAVE project (only for QTRP)

## Note
The codes in this branch should be put on QTRP (/home/qtrobot/catkin_ws/src) so that the ros block in the Blockly can handle the custom messge of type migrave_games/TaskParameters.msg. 

This message type is defined so that the emotion, images, etc. in the task of the emotion game can be sent in one message to speed up the game. 

The codes in main branch is the actual codes for the games.

## Project Structure
```
├── CMakeLists.txt
├── README.md
├── migrave_games
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── TaskParameters.msg
│   └── package.xml
└── msg
    └── TaskParameters.msg
```
