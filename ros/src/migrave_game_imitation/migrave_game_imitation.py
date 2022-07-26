#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from migrave_game_library.game_base import GameBase

class MigraveGameImitation(GameBase):
    def __init__(self, game_config_dir_path: str,
                 game_id: str,
                 game_status_topic: str,
                 game_answer_topic: str,
                 game_performance_topic: str):
        super(MigraveGameImitation, self).__init__(game_config_dir_path,
                                                   game_id,
                                                   game_status_topic,
                                                   game_answer_topic,
                                                   game_performance_topic)

        self.gestures = self.game_config["game_specific_params"]["gestures"]
        self.gesture_texts = self.game_config["game_specific_params"]["gesture_texts"]
        self.retrieval_gestures = self.game_config["game_specific_params"]["retrieval_gestures"]

    def game_start(self):
        super().game_start()

        rospy.loginfo("Game starts!")
        self.say_text("Jetzt üben wir Bewegungen nachmachen.")
        self.say_text("Das Lerner-Tablet bekommt dehr, oder die, Erwachsene.")
        self.show_emotion("showing_smile")
        self.say_text("Los geht's!")
        self.show_emotion("showing_smile")

    def task_start(self):
        super().task_start()

        self.say_text("Hände auf den Tisch, schau mich an.")
        self.say_text("Ich mach vor und du schaust zu! Danach bist du dran!")
        self.say_text(self.gesture_texts[self.task])
        self.gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.gesture_play(self.retrieval_gestures[self.task])
        self.say_text("Du bist dran. Mach nach!")

        # Show choices (right, almost right, wrong) on the educator tablet
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")

    def evaluate_answer(self):
        feedback_emotions = {
            "right": "showing_smile",
            "right_after_wrong": "showing_smile",
            "wrong": "",
            "wrong_again": "",
        }
        right_texts = {
            "hands-up": "Richtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
            "hands-up-resume": "Richtig, Arme hoch! Wunderbar!",
            "hands-side-resume": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly-resume": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        right_after_wrong_texts = {
            "hands-up": "Richtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
            "hands-up-resume": "Richtig, Arme hoch! Wunderbar!",
            "hands-side-resume": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly-resume": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        feedback_texts = {
            "right": right_texts[self.task],
            "right_after_wrong": right_after_wrong_texts[self.task],
            "wrong": "Schau nochmal genau hin!",
            "wrong_again": "Schau nochmal genau hin!",
        }

        super().evaluate_answer(feedback_emotions, feedback_texts)

    def game_status_cb(self, msg):
        super().game_status_cb(msg)

        # Clear the picture on the tablet once the game is over
        if "end" in self.game_status:
            rospy.sleep(2)
            rospy.loginfo("Clearing learner tablet image")
            self.tablet_image_pub.publish("Nix")

    def retry_after_wrong(self):
        self.say_text(self.gesture_texts[self.task])
        self.gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.gesture_play(self.retrieval_gestures[self.task])
        self.say_text("Du bist dran. Mach nach!")
        self.task_status_pub.publish("Running")
        rospy.loginfo("Publish task status: Running")
        rospy.sleep(1)
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")
        rospy.loginfo("Wait for grading")

    def start_new_round_and_grade(self):
        self.say_text(self.gesture_texts[self.task])
        self.gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.gesture_play(self.retrieval_gestures[self.task])
        self.say_text("Du bist dran. Mach nach!")
        self.task_status_pub.publish("Running")
        rospy.loginfo("Publish task status: Running")
        rospy.sleep(1)
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")
        rospy.loginfo("Wait for grading")
