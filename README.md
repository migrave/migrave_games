# migrave_games
Repository of games designed for the MigrAVE project. 

## Introduction
The repository contains two games: 1) Imitation game and 2) Emotions game. Each game consists of a ROS node and a corresponding Blockly program (created using QTrobot Studio). This mixed structure is adopted such that the the Blockly game can be more compact. Each game can be indeed fully implemented as a Blockly game, but it will contain too many blocks, making it difficult to debug and maintain. In addition, the ROS node provides more flexibility when designing the games. 

### Imitation game

- ROS node: ./ros/scripts/migrave_game_imitation
- Blockly game: rfh-koeln/MIGRAVE/ROS/Imitation-v2

<!--- 
![Imitation](https://i.ibb.co/wKystWK/imitation.png)
-->

### Emotions game

- ROS node: ./ros/scripts/migrave_game_emotions
- Blockly game: rfh-koeln/MIGRAVE/ROS/Emotions-v2

## Usage

Launch the ROS nodes

```sh
roslaunch migrave_games migrave_games.launch
```

Use the Educator app in the Educator tablet to choose one game and run it. 
