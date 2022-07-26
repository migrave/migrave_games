#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
import rospy

from migrave_game_library.game_base import GameBase
from migrave_games.msg import TaskParameters

class MigraveGameEmotions(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str = "/migrave_game_emotions/status",
                 game_answer_topic: str = "/migrave_game_emotions/answer",
                 game_performance_topic: str = "/migrave/game_performance"):
        super(MigraveGameEmotions, self).__init__(game_config_dir_path,
                                                  game_id,
                                                  game_status_topic,
                                                  game_answer_topic,
                                                  game_performance_topic)

        self.images_happy = self.game_config["game_specific_params"]["happy_images"]
        self.images_sad = self.game_config["game_specific_params"]["sad_images"]

        # self.game_performance_topic = rospy.get_param("~performance_topic", "/migrave/game_performance")
        self.task_parameters = TaskParameters()
        self.task_parameters_pub = rospy.Publisher("/migrave_game_emotions/task_parameters",
                                                   TaskParameters, queue_size=1)
        self.emotion = "Waiting"
        self.emotion_image = "Nix"

    def game_start(self):
        super().game_start()

        rospy.loginfo("Emotion game starts!")
        self.say_text("Jetzt üben wir Gefühle erkennen. Los geht's!")
        self.show_emotion("showing_smile")
        self.say_text("Hände auf den Tisch, schau mich an. Los geht's!")
        self.show_emotion("showing_smile")
        self.say_text("Ich nenne dir ein Gefühl. Du tippst auf das passende Bild.")

    def task_start(self):
        super().task_start()

        # Show image
        if self.task in ["happy", "sad", "happy_resume", "sad_resume"]:
            rospy.loginfo("Simple task")
            self.start_new_round_simple()
        if self.task in ["happy_vs_sad", "sad_vs_happy",
                         "happy_or_sad", "happy_vs_sad_resume",
                         "sad_vs_happy_resume", "happy_or_sad_resume"]:
            rospy.loginfo("Normal task")
            self.start_new_round()

    def evaluate_answer(self):
        feedback_emotions = {
            "right": "showing_smile",
            "right_1": "showing_smile",
            "right_2": "showing_smile",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": "",
        }
        right_texts = {
            "happy": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "happy_vs_sad": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad_vs_happy": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "sad_or_happy": f"Richtig, die Person zeigt ein {self.emotion}s Gesicht. Wunderbar!",
            "happy_resume": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad_resume": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "happy_vs_sad_resume": "Richtig, die Person zeigt ein glückliches Gesicht. Wunderbar!",
            "sad_vs_happy_resume": "Richtig, die Person zeigt ein trauriges Gesicht. Wunderbar!",
            "sad_or_happy_resume": f"Richtig, die Person zeigt ein {self.emotion}s Gesicht. Wunderbar!",
        }
        feedback_texts = {
            "right": right_texts[self.task],
            "right_1": right_texts[self.task],
            "right_2": right_texts[self.task],
            "wrong": "Schau nochmal genau hin!",
            "wrong_1": "Schau nochmal genau hin!",
            "wrong_2": "Schau nochmal genau hin!",
        }

        super().evaluate_answer(feedback_emotions, feedback_texts)

    def retry_after_wrong(self):
        self.task_status_pub.publish("running")
        rospy.loginfo("Publish task status: running")
        rospy.sleep(2)

    def start_new_round_and_grade(self):
        self.task_status_pub.publish("running")
        rospy.loginfo("Publish task status: running")
        rospy.sleep(2)

        if self.task in ["happy", "sad", "happy_resume", "sad_resume"]:
            self.start_new_round_simple()

        if self.task in ["happy_vs_sad", "sad_vs_happy",
                         "happy_or_sad", "happy_vs_sad_resume",
                         "sad_vs_happy_resume", "happy_or_sad_resume"]:
            self.start_new_round()

    def start_new_round_simple(self):
        if self.task in ["happy", "happy_resume"]:
            self.emotion = "glückliche"
            self.emotion_image = random.choice(self.images_happy)
        if self.task in ["sad", "sad_resume"]:
            self.emotion = "traurige"
            self.emotion_image = random.choice(self.images_sad)

        self.say_text("Schau auf das Tablet!")
        self.task_parameters.emotion = self.emotion
        rospy.loginfo(f"Correct emotion: {self.emotion}")
        self.task_parameters.image_1 = self.emotion_image
        rospy.loginfo(f"Correct image: {self.emotion_image}")
        self.task_parameters_pub.publish(self.task_parameters)
        rospy.loginfo(f"Publish: {self.emotion}, {self.emotion_image}")

    def start_new_round(self):
        self.image_happy = random.choice(self.images_happy)
        self.image_sad = random.choice(self.images_sad)

        # choose the hightligthed correct image (e.g. Happy1X)
        if self.task in ["happy_vs_sad", "happy_vs_sad_resume"]:
            self.image_x = f"{self.image_happy}X"
        if self.task in ["sad_vs_happy", "sad_vs_happy_resume"]:
            self.image_x = f"{self.image_sad}X"

        self.images = [self.image_happy, self.image_sad]
        random.shuffle(self.images)

        # find the order of the correct image
        # and set the German word for happy or sad
        self.correct_image = 2
        if self.task in ["happy_vs_sad", "happy_vs_sad_resume"]:
            if "Happy" in self.images[0]:
                self.correct_image = 1
            self.emotion = "glückliche"

        if self.task in ["sad_vs_happy", "sad_vs_happy_resume"]:
            if "Sad" in self.images[0]:
                self.correct_image = 1
            self.emotion = "traurige"

        # Not used currently due to time constraint
        if self.task in ["happy_or_sad", "happy_or_sad_resume"]:
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

        rospy.loginfo(f"Images: {self.images}")
        rospy.loginfo(f"Order of the correct image: {self.correct_image}")
        rospy.loginfo(f"Correct emotion: {self.emotion}")

        self.say_text("Schau auf das Tablet!")

        self.task_parameters.emotion = self.emotion
        self.task_parameters.image_1 = self.images[0]
        self.task_parameters.image_2 = self.images[1]
        self.task_parameters.image_x = self.image_x
        self.task_parameters.correct_image = str(self.correct_image)
        self.task_parameters_pub.publish(self.task_parameters)
