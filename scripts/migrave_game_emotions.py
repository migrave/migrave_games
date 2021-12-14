#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
import random


class migrave_game_emotions:
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
            "/migrave_game_emotions/status", String, queue_size=10
        )
        self.tablet_image_pub = rospy.Publisher(
            "/migrave_game_emotions/tablet_image", String, queue_size=10
        )
        self.show_educator_choice_pub = rospy.Publisher(
            "/migrave_game_emotions/show_educator_choice", Bool, queue_size=10
        )
        self.emotion_pub = rospy.Publisher(
            "/migrave_game_emotions/emotion", String, queue_size=10
        )

        self.correct_image_pub = rospy.Publisher(
            "/migrave_game_emotions/correct_image", String, queue_size=10
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
        self.emotion_image = "Nix"

        self.tasks = ["happy", "sad", "happy_vs_sad",
                      "sad_vs_happy", "happy_or_sad"]
        self.images_happy = ["Happy1", "Happy2"]
        self.images_sad = ["Sad1", "Sad2"]
        self.images_mixed = [
            ["Happy1", "Sad1"],
            ["Sad2", "Happy2"],
            ["Happy2", "Sad1"],
            ["Sad2", "Happy1"],
        ]

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
            self.talkText_pub.publish("Jetzt üben wir Gefühle erkennen.")
            rospy.sleep(2)
            self.talkText_pub.publish("Los geht's!")
            rospy.sleep(3)
            self.emotionShow_pub.publish("showing_smile")
            rospy.sleep(2)
            self.talkText_pub.publish(
                "Hände auf den Tisch, schau mich an. Los geht's!")
            rospy.sleep(2)
            self.emotionShow_pub.publish("showing_smile")
            rospy.sleep(2)
            self.talkText_pub.publish(
                "Ich nenne dir ein Gefühl. Du tippst auf das passende Bild."
            )
            rospy.sleep(6)
            rospy.loginfo("End of intro")

    def task_start(self):
        if self.game_status in self.tasks:
            self.count = 0
            self.correct = 0
            self.talkText_pub.publish("Schau auf das Tablet!")
            rospy.sleep(2)
            self.task = self.game_status
            rospy.loginfo(f"Task: {self.task}")

            # Update game status
            self.game_status = "Running"
            self.game_status_pub.publish("Running")
            rospy.loginfo("Publish status: Running")
            rospy.sleep(2)

            # Show image
            if self.task == "happy":
                self.emotion = "glückliche"
                self.emotion_image = random.choice(self.images_happy)
                rospy.loginfo(f"Show image: {self.emotion_image}")
                self.tablet_image_pub.publish(self.emotion_image)
                rospy.sleep(2)

            if self.task == "sad":
                self.emotion = "traurige"
                self.emotion_image = random.choice(self.images_sad)
                rospy.loginfo(f"Show image: {self.emotion_image}")
                self.tablet_image_pub.publish(self.emotion_image)
                rospy.sleep(2)

            if self.task == "happy_vs_sad":
                self.image_happy = random.choice(self.images_happy)
                self.image_x = f"{self.image_happy}X"
                self.image_sad = random.choice(self.images_sad)
                self.images = [self.image_happy, self.image_sad]
                random.shuffle(self.images)
                self.correct_image = 2
                if "Happy" in self.images[0]:
                    self.correct_image = 1
                self.emotion = "glückliche"
                rospy.loginfo(self.images)
                rospy.loginfo(self.correct_image)
                self.tablet_image_pub.publish(self.images[0])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.images[1])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.image_x)
                rospy.sleep(1)
                self.emotion_pub.publish(self.emotion)
                rospy.sleep(1)
                self.correct_image_pub.publish(str(self.correct_image))
                rospy.sleep(1)

            if self.task == "sad_vs_happy":
                self.image_happy = random.choice(self.images_happy)
                self.image_sad = random.choice(self.images_sad)
                self.image_x = f"{self.image_sad}X"
                self.images = [self.image_happy, self.image_sad]
                random.shuffle(self.images)
                self.correct_image = 2
                if "Sad" in self.images[0]:
                    self.correct_image = 1
                self.emotion = "traurige"
                rospy.loginfo(self.images)
                rospy.loginfo(self.correct_image)
                self.tablet_image_pub.publish(self.images[0])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.images[1])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.image_x)
                rospy.sleep(1)
                self.emotion_pub.publish(self.emotion)
                rospy.sleep(1)
                self.correct_image_pub.publish(str(self.correct_image))
                rospy.sleep(1)

            if self.task == "happy_or_sad":
                self.image_happy = random.choice(self.images_happy)
                self.image_sad = random.choice(self.images_sad)
                self.emotion = random.choice["glückliche", "traurige"]
                if self.emotion == "glückliche":
                    self.image_x = f"{self.image_happy}X"
                    self.images = [self.image_happy, self.image_sad]
                    random.shuffle(self.images)
                    self.correct_image = 2
                    if "Happy" in self.images[0]:
                        self.correct_image = 1
                if self.emotion == "traurige":
                    self.image_x = f"{self.image_sad}X"
                    self.images = [self.image_happy, self.image_sad]
                    random.shuffle(self.images)
                    self.correct_image = 2
                    if "Sad" in self.images[0]:
                        self.correct_image = 1

                rospy.loginfo(self.emotion)
                rospy.loginfo(self.images)
                rospy.loginfo(self.correct_image)
                self.tablet_image_pub.publish(self.images[0])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.images[1])
                rospy.sleep(1)
                self.tablet_image_pub.publish(self.image_x)
                rospy.sleep(1)
                self.emotion_pub.publish(self.emotion)
                rospy.sleep(1)
                self.correct_image_pub.publish(str(self.correct_image))
                rospy.sleep(1)

    def game_grade(self):
        result = self.result
        # Feedback after grading
        feedback_emotions = {
            "Right": "showing_smile",
            "Right_1": "showing_smile",
            "Right_2": "showing_smile",
            "Almost_right": "showing_smile",
            "Wrong": "",
            "Wrong_1": "",
            "Wrong_2": "",
            "Wrong_3": "",
        }
        right_texts = {
            "happy": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "happy_vs_sad": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad_vs_happy": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "sad_or_happy": f"Richtig, die Person zeigt ein {self.emotion}s Gesicht. Wunderbar!",
        }
        almost_right_texts = {
            "happy": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "happy_vs_sad": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad_vs_happy": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "sad_or_happy": f"Richtig, die Person zeigt ein {self.emotion}s Gesicht. Wunderbar!",
        }
        feedback_texts = {
            "Right": right_texts[self.task],
            "Right_1": right_texts[self.task],
            "Right_2": right_texts[self.task],
            "Almost_right": almost_right_texts[self.task],
            "Wrong": "Schau nochmal genau hin!",
            "Wrong_1": "Schau nochmal genau hin!",
            "Wrong_2": "Schau nochmal genau hin!",
            "Wrong_3": "Schau nochmal genau hin!",
        }
        feedback_gestures = {
            "Right": "QT/clapping",
            "Right_1": "QT/clapping",
            "Right_2": "QT/clapping",
            "Almost_right": "",
            "Wrong": "",
            "Wrong_1": "",
            "Wrong_2": "",
            "Wrong_3": "",
        }

        if self.count < 5:
            # Reaction after grading
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

                # Start over when only correct 3 times in the first 5 rounds
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

                if self.count < 5:
                    rospy.sleep(2)
                    self.start_new_round_and_grade()

                if self.count == 5 and self.correct <= 3:
                    self.count = 0
                    self.correct = 0
                    rospy.sleep(1)
                    self.talkText_pub.publish(
                        "Lass uns die Bewegung nochmal üben!")
                    rospy.sleep(4)
                    self.start_new_round_and_grade()

            # For happy_vs_sad
            if result == "Right_1" or result == "Right_2":
                self.emotionShow_pub.publish("showing_smile")
                rospy.sleep(2)
                self.talkText_pub.publish("Noch ein mal!")
                rospy.sleep(2)
                self.start_new_round_and_grade()

            if result == "Wrong_1":
                self.count += 1
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")

                if self.count == 5 and self.correct <= 3:
                    self.count = 0
                    self.correct = 0
                    rospy.sleep(1)
                    self.talkText_pub.publish(
                        "Lass uns die Bewegung nochmal üben!")
                    rospy.sleep(4)
            if result == "Wrong_2" or result == "Wrong_3":
                rospy.loginfo(f"{result}")

        else:
            rospy.sleep(2)
            # self.gesturePlay_pub.publish("QT/imitation/hands-up-back")
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
                rospy.loginfo("54 case")
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
                elif result == "Almost_right" or "Right_1" or "Right_2":
                    self.emotionShow_pub.publish("showing_smile")
                    rospy.sleep(2)
                    self.talkText_pub.publish("Noch ein mal!")
                    rospy.sleep(2)
                    self.start_new_round_and_grade()
                elif result == "Wrong_1" or "Wrong_2" or "Wrong_3":
                    rospy.loginfo(f"{result}")
                else:  # Wrong
                    self.start_new_round_and_grade()

        if self.correct == 5:
            rospy.loginfo("Endding")
            rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")
            self.finish_one_task()

    def start_new_round_and_grade(self):
        self.game_status_pub.publish("Running")
        rospy.loginfo("Publish status: Running")
        rospy.sleep(2)
        self.talkText_pub.publish("Schau auf das Tablet!")
        rospy.sleep(2)

        # Show image
        if self.task == "happy":
            self.emotion_image = random.choice(self.images_happy)
            rospy.loginfo(f"Show image: {self.emotion_image}")
            self.tablet_image_pub.publish(self.emotion_image)
            rospy.sleep(2)
        if self.task == "sad":
            self.emotion_image = random.choice(self.images_sad)
            rospy.loginfo(f"Show image: {self.emotion_image}")
            self.tablet_image_pub.publish(self.emotion_image)
            rospy.sleep(2)

        if self.task == "happy_vs_sad":
            self.image_happy = random.choice(self.images_happy)
            self.image_x = f"{self.image_happy}X"
            self.image_sad = random.choice(self.images_sad)
            self.images = [self.image_happy, self.image_sad]
            random.shuffle(self.images)
            self.correct_image = 2
            if "Happy" in self.images[0]:
                self.correct_image = 1
            self.emotion = "glückliche"
            rospy.loginfo(self.images)
            rospy.loginfo(self.correct_image)
            self.tablet_image_pub.publish(self.images[0])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.images[1])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.image_x)
            rospy.sleep(1)
            self.emotion_pub.publish(self.emotion)
            rospy.sleep(1)
            self.correct_image_pub.publish(str(self.correct_image))
            rospy.sleep(1)

        if self.task == "sad_vs_happy":
            self.image_happy = random.choice(self.images_happy)
            self.image_sad = random.choice(self.images_sad)
            self.image_x = f"{self.image_sad}X"
            self.images = [self.image_happy, self.image_sad]
            random.shuffle(self.images)
            self.correct_image = 2
            if "Sad" in self.images[0]:
                self.correct_image = 1
            self.emotion = "traurige"
            rospy.loginfo(self.images)
            rospy.loginfo(self.correct_image)
            self.tablet_image_pub.publish(self.images[0])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.images[1])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.image_x)
            rospy.sleep(1)
            self.emotion_pub.publish(self.emotion)
            rospy.sleep(1)
            self.correct_image_pub.publish(str(self.correct_image))
            rospy.sleep(1)

        if self.task == "happy_or_sad":
            self.image_happy = random.choice(self.images_happy)
            self.image_sad = random.choice(self.images_sad)
            self.emotion = random.choice["glückliche", "traurige"]
            if self.emotion == "glückliche":
                self.image_x = f"{self.image_happy}X"
                self.images = [self.image_happy, self.image_sad]
                random.shuffle(self.images)
                self.correct_image = 2
                if "Happy" in self.images[0]:
                    self.correct_image = 1
            if self.emotion == "traurige":
                self.image_x = f"{self.image_sad}X"
                self.images = [self.image_happy, self.image_sad]
                random.shuffle(self.images)
                self.correct_image = 2
                if "Sad" in self.images[0]:
                    self.correct_image = 1

            rospy.loginfo(self.emotion)
            rospy.loginfo(self.images)
            rospy.loginfo(self.correct_image)
            self.tablet_image_pub.publish(self.images[0])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.images[1])
            rospy.sleep(1)
            self.tablet_image_pub.publish(self.image_x)
            rospy.sleep(1)
            self.emotion_pub.publish(self.emotion)
            rospy.sleep(1)
            self.correct_image_pub.publish(str(self.correct_image))
            rospy.sleep(1)

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

    game_status_topic = "/migrave_game_emotions/status"
    game_answer_topic = "/migrave_game_emotions/answer"
    game = migrave_game_emotions(game_status_topic, game_answer_topic)
    rospy.init_node("migrave_game_emotions", anonymous=True)
    rospy.spin()
