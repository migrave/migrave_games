#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool


class migrave_game_imitation:
    def __init__(self, game_status_topic, game_answer_topic):
        self.game_status_topic = game_status_topic
        self.game_answer_topic = game_answer_topic

        # QT built-in topic publishers
        self.talkText_pub = rospy.Publisher(
            "/qt_robot/behavior/talkText", String, queue_size=10
        )

        self.talkAudio_pub = rospy.Publisher(
            "/qt_robot/behavior/talkAudio", String, queue_size=10
        )

        self.gesturePlay_pub = rospy.Publisher(
            "/qt_robot/gesture/play", String, queue_size=10
        )

        self.emotionShow_pub = rospy.Publisher(
            "/qt_robot/emotion/show", String, queue_size=10
        )

        # Game related topic publishers
        self.game_status_pub = rospy.Publisher(
            "/migrave_game_imitation/status", String, queue_size=10
        )
        self.tablet_image_pub = rospy.Publisher(
            "/migrave_game_imitation/tablet_image", String, queue_size=10
        )
        self.show_educator_choice_pub = rospy.Publisher(
            "/migrave_game_imitation/show_educator_choice", Bool, queue_size=10
        )

        # Game related topic subscribers
        self.game_status_subscriber = rospy.Subscriber(
            self.game_status_topic, String, self.game_status_callback
        )

        self.game_answer_subscriber = rospy.Subscriber(
            self.game_answer_topic, String, self.game_answer_callback
        )

        # Initialization
        self.count = 0
        self.correct = 0
        self.redo_result = None
        self.regraded = False
        self.game_status = "Waiting"
        self.result = "Waiting"
        self.task = "Waiting"

        self.tasks = ["hands-up", "hands-side", "Fly"]
        self.texts = {
            "hands-up": "Beide Arme nach oben!",
            "hands-side": "Beide Arme zur Seite strecken!",
            "Fly": "Arme seitlich rauf und runter!",
        }
        self.gestures = {
            "hands-up": "QT/imitation/hands-up",
            "hands-side": "QT/imitation/hands-side",
            "Fly": "QT/Pretend-play/Fly",
        }
        self.gestures_restore = {
            "hands-up": "QT/imitation/hands-up-back",
            "hands-side": "QT/imitation/hands-side-back",
            "Fly": "",
        }

    def game_status_callback(self, msg):
        self.game_status = msg.data
        self.game_start()
        self.task_start()

    def game_answer_callback(self, msg):
        rospy.loginfo("game answer callback")
        self.result = msg.data
        rospy.loginfo(f"Game result: {self.result}")
        self.game_grade()

    def game_start(self):
        if self.game_status == "Start":
            self.count = 0
            self.correct = 0
            # Introduction
            rospy.loginfo("Game starts!")
            rospy.loginfo("Start intro")
            self.talkText_pub.publish("Jetzt üben wir Bewegungen nachmachen.")
            rospy.sleep(2)
            self.talkText_pub.publish(
                "Das Lerner-Tablet bekommt dehr, oder die, Erwachsene"
            )
            rospy.sleep(3)
            self.emotionShow_pub.publish("showing_smile")
            rospy.sleep(2)
            self.talkText_pub.publish("Los geht's!")
            rospy.sleep(2)
            self.emotionShow_pub.publish("showing_smile")
            rospy.sleep(2)
            rospy.loginfo("End of intro")

    def task_start(self):
        if self.game_status in self.tasks:
            self.count = 0
            self.correct = 0
            self.task = self.game_status
            self.talkText_pub.publish("Hände auf den Tisch, schau mich an.")
            rospy.sleep(2)
            self.talkText_pub.publish("Ich mach vor und du machst nach!")
            rospy.sleep(2)
            self.talkText_pub.publish(self.texts[self.task])
            rospy.sleep(2)
            self.gesturePlay_pub.publish(self.gestures[self.task])
            rospy.sleep(6)
            # Update game status
            self.game_status = "Running"
            self.game_status_pub.publish("Running")
            rospy.loginfo("Publish status: Running")
            rospy.sleep(6)
            # Show choices (Right, Almost right, Wrong) on Educator Tablet
            self.show_educator_choice_pub.publish(True)
            rospy.loginfo("Publish choice")

    def game_grade(self):
        # Feedback after grading
        feedback_emotions = {
            "Right": "showing_smile",
            "Almost_right": "showing_smile",
            "Wrong": "",
        }
        right_texts = {
            "hands-up": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        almost_right_texts = {
            "hands-up": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        feedback_texts = {
            "Right": right_texts[self.task],
            "Almost_right": almost_right_texts[self.task],
            "Wrong": "Schau nochmal genau hin!",
        }
        feedback_gestures = {
            "Right": "QT/clapping",
            "Almost_right": "",
            "Wrong": "",
        }
        result = self.result

        if self.count < 5:
            # Reaction after grading
            # rospy.sleep(2)
            # self.gesturePlay_pub.publish("QT/imitation/hands-up-back")
            self.gesturePlay_pub.publish(self.gestures_restore[self.task])
            rospy.sleep(2)
            emotion = feedback_emotions[result]
            self.emotionShow_pub.publish(emotion)
            text = feedback_texts[result]
            self.talkText_pub.publish(text)
            rospy.sleep(2)
            gesture = feedback_gestures[result]
            self.gesturePlay_pub.publish(gesture)
            rospy.sleep(6)

            if self.result == "Right":
                self.count += 1
                self.correct += 1
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")

                # Restart if correct no more than 3 times in the first 5 rounds
                if self.count == 5 and self.correct <= 3:
                    self.count = 0
                    self.correct = 0
                    self.talkText_pub.publish("Neue Runde")
                    rospy.sleep(2)
                    rospy.loginfo("Restart the game")
                    self.start_new_round_and_grade()

                # Finish the task when correct 5 times in the first 5 rounds
                elif self.count == 5 and self.correct == 5:
                    self.talkText_pub.publish(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    rospy.sleep(4)
                    self.talkAudio_pub.publish("rfh-koeln/MIGRAVE/Reward2")
                    rospy.sleep(2)
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)
                    self.tablet_image_pub.publish(image)
                    rospy.sleep(6)

                # For other cases, start a new round
                else:
                    rospy.loginfo("Continue")
                    self.talkText_pub.publish(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    rospy.sleep(4)
                    self.talkAudio_pub.publish("rfh-koeln/MIGRAVE/Reward2")
                    rospy.sleep(2)
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)

                    self.tablet_image_pub.publish(image)
                    rospy.loginfo(f"Publish image: {self.correct}Token")
                    rospy.sleep(6)

                    self.emotionShow_pub.publish("showing_smile")
                    rospy.sleep(2)
                    self.talkText_pub.publish("Noch ein mal!")
                    rospy.sleep(2)

                    self.start_new_round_and_grade()

            # if almost right, start a new round
            if result == "Almost_right":
                self.emotionShow_pub.publish("showing_smile")
                rospy.sleep(2)
                self.talkText_pub.publish("Noch ein mal!")
                rospy.sleep(2)
                self.start_new_round_and_grade()

            if result == "Wrong":
                self.count += 1
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")

                # Restart if correct no more than 3 times in the first 5 rounds
                if self.count == 5 and self.correct <= 3:
                    self.count = 0
                    self.correct = 0
                    rospy.sleep(1)
                    self.talkText_pub.publish(
                        "Lass uns die Bewegung nochmal üben!")
                    rospy.sleep(4)
                    self.start_new_round_and_grade()
                else:  # next round for other cases
                    rospy.sleep(2)
                    self.start_new_round_and_grade()
                # if self.count < 5:
                #     rospy.sleep(2)
                #     self.start_new_round_and_grade()

        else:
            rospy.sleep(2)
            # self.gesturePlay_pub.publish("QT/imitation/hands-up-back")
            self.gesturePlay_pub.publish(self.gestures_restore[self.task])
            rospy.sleep(4)
            emotion = feedback_emotions[result]
            self.emotionShow_pub.publish(emotion)
            text = feedback_texts[result]
            self.talkText_pub.publish(text)
            rospy.sleep(2)
            gesture = feedback_gestures[result]
            self.gesturePlay_pub.publish(gesture)
            rospy.sleep(6)
            if self.count == 5 and self.correct == 4:
                rospy.loginfo("80% correctness case")
                if result == "Right":
                    self.count += 1
                    self.correct += 1
                    rospy.loginfo(
                        f"Count: {self.count}; Correct: {self.correct}")
                    self.talkText_pub.publish(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    rospy.sleep(4)
                    self.talkAudio_pub.publish("rfh-koeln/MIGRAVE/Reward2")
                    rospy.sleep(2)
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)

                    self.tablet_image_pub.publish(image)
                    rospy.loginfo(f"Publish image: {self.correct}Token")
                    rospy.sleep(6)
                elif result == "Almost_right":
                    self.emotionShow_pub.publish("showing_smile")
                    rospy.sleep(2)
                    self.talkText_pub.publish("Noch ein mal!")
                    rospy.sleep(2)
                    self.start_new_round_and_grade()
                else:  # Wrong
                    self.start_new_round_and_grade()

        if self.correct == 5:
            rospy.loginfo("Endding")
            rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")
            self.finish_one_task()

    def start_new_round_and_grade(self):
        self.talkText_pub.publish("Mach nach!")
        rospy.sleep(2)
        # self.talkText_pub.publish("Beide Arme nach oben.")
        # self.gesturePlay_pub.publish("QT/imitation/hands-up")
        self.talkText_pub.publish(self.texts[self.task])
        self.gesturePlay_pub.publish(self.gestures[self.task])
        rospy.sleep(4)
        self.game_status_pub.publish("Running")
        rospy.loginfo("Publish status: Running")
        rospy.sleep(6)
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")
        rospy.loginfo("Wait for grading")

    def finish_one_task(self):
        self.emotionShow_pub.publish("showing_smile")
        rospy.sleep(1)
        self.talkText_pub.publish("Geschafft! Das hast du super gemacht!")
        rospy.sleep(3)
        self.gesturePlay_pub.publish("QT/Dance/Dance-1-1")
        rospy.sleep(6)
        self.emotionShow_pub.publish("showing_smile")
        rospy.sleep(2)
        self.gesturePlay_pub.publish("QT/imitation/hands-up-back")
        rospy.sleep(4)
        self.game_status_pub.publish("Finish")
        rospy.loginfo("Publish status: Finish")
        rospy.sleep(4)
        self.talkText_pub.publish(
            "Schau mal auf das Tablet. Da ist ein Feuerwerk für dich!"
        )
        rospy.sleep(2)
        self.tablet_image_pub.publish("Fireworks")
        rospy.loginfo("Publish image: Fireworks")
        # rospy.sleep(2)
        self.talkAudio_pub.publish("rfh-koeln/MIGRAVE/Fireworks")
        rospy.sleep(20)
        self.tablet_image_pub.publish("Nix")
        rospy.loginfo("Publish image: Nix")
        self.count = 0
        self.correct = 0


if __name__ == "__main__":

    game_status_topic = "/migrave_game_imitation/status"
    game_answer_topic = "/migrave_game_imitation/answer"
    game = migrave_game_imitation(game_status_topic, game_answer_topic)
    rospy.init_node("migrave_game_imitation", anonymous=True)
    rospy.spin()
