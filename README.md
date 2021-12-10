# migrave_games
Repository of games designed for the MigrAVE project. 

Each game consists of a ROS node and a corresponding Blockly game (created using QTrobot Studio). This mixed structure is adopted such that the the Blockly game can be more compact. Each game can be indeed fully implemented as a Blockly game, but it will contain too many blocks, making it difficult to debug and maintain. In addition, the ROS node provide more flexibility when designing the games. 

## Imitation game

- ROS node: migrave_game_imitation.py
- Blockly game: rfh-koeln/MIGRAVE/ROS/Imitation

![Imitation](https://ixpduesseldorf-my.sharepoint.com/:i:/g/personal/rainer_speicher_ixpduesseldorf_onmicrosoft_com/EfzVIYUbYuBAi6LJepFZ244BfkaSuk05rRi0aQPJUSYmlw?e=XIo59O)

