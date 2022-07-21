# migrave_games
Repository of games designed for the MigrAVE project. 

## Introduction
The repository contains two games: 1) Imitation game and 2) Emotions game. Each game consists of a ROS node and a corresponding Blockly program (created using QTrobot Studio). This mixed structure is adopted such that the the Blockly game can be more compact. Each game can be indeed fully implemented as a Blockly game, but it will contain too many blocks, making it difficult to debug and maintain. In addition, the ROS node provides more flexibility when designing the games. 

__Note__: There are two branches. The `main` branch codes go to QTPC, and the `QTRP` branch codes go to QTRP.

### Imitation game

- ROS node: ./ros/scripts/migrave_game_imitation
- Blockly game: rfh-koeln/MIGRAVE/ROS/Imitation-v3

<!--- 
![Imitation](https://i.ibb.co/wKystWK/imitation.png)
-->

### Emotions game

- ROS node: ./ros/scripts/migrave_game_emotions
- Blockly game: rfh-koeln/MIGRAVE/ROS/Emotions-v3

## Usage

Launch the ROS nodes

```sh
roslaunch migrave_games migrave_games.launch
```

Use the Educator app in the Educator tablet to choose one game and run it. 

## Project Structure

```
.
├── CMakeLists.txt
├── README.md
├── package.xml
├── ros
│   ├── launch
│   │   └── migrave_games.launch
│   ├── msg
│   │   └── TaskParameters.msg
│   ├── scripts
│   │   ├── migrave_game_emotions
│   │   └── migrave_game_imitation
│   └── src
│       ├── migrave_game_emotions
│       │   ├── __init__.py
│       │   └── migrave_game_emotions.py
│       └── migrave_game_imitation
│           ├── __init__.py
│           └── migrave_game_imitation.py
└── setup.py
```
