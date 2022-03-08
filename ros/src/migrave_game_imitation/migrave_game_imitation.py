#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
from migrave_ros_msgs.msg import GamePerformance
from qt_robot_interface.srv import (
    behavior_talk_text,
    emotion_show,
    audio_play,
)
from qt_gesture_controller.srv import gesture_play


class MigraveGameImitation:
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
        self.task_status_pub = rospy.Publisher(
            "/migrave_game_imitation/task_status", String, queue_size=10
        )
        self.tablet_image_pub = rospy.Publisher(
            "/migrave_game_imitation/tablet_image", String, queue_size=10
        )
        self.show_educator_choice_pub = rospy.Publisher(
            "/migrave_game_imitation/show_educator_choice", Bool, queue_size=10
        )

        # Game related topic subscribers
        self.game_status_sub = rospy.Subscriber(
            self.game_status_topic, String, self.game_status_callback
        )

        self.game_answer_sub = rospy.Subscriber(
            self.game_answer_topic, String, self.game_answer_callback
        )

        self.game_performance_topic = rospy.get_param(
            "~performance_topic", "/migrave/game_performance"
        )

        self.game_performance_subscriber = rospy.Subscriber(
            self.game_performance_topic, GamePerformance, self.game_performance_callback
        )

        self.game_performance_pub = rospy.Publisher(
            self.game_performance_topic, GamePerformance, queue_size=1
        )

        # Initialization
        # robot setting
        self.gesture_speed = 1.0
        # game parameters
        self.count = 0
        self.correct = 0
        self.game_status = "waiting"
        self.task_status = "waiting"
        self.result = "waiting"
        self.task = "waiting"

        self.tasks = [
            "hands-up",
            "hands-side",
            "Fly",
            "hands-up-resume",
            "hands-side-resume",
            "Fly-resume",
        ]
        self.texts = {
            "hands-up": "Beide Arme nach oben!",
            "hands-side": "Beide Arme zur Seite strecken!",
            "Fly": "Arme seitlich rauf und runter!",
            "hands-up-resume": "Beide Arme nach oben!",
            "hands-side-resume": "Beide Arme zur Seite strecken!",
            "Fly-resume": "Arme seitlich rauf und runter!",
        }
        self.gestures = {
            "hands-up": "QT/imitation/hands-up",
            "hands-side": "QT/imitation/hands-side",
            "Fly": "QT/Pretend-play/Fly",
            "hands-up-resume": "QT/imitation/hands-up",
            "hands-side-resume": "QT/imitation/hands-side",
            "Fly-resume": "QT/Pretend-play/Fly",
        }
        self.gestures_restore = {
            "hands-up": "QT/imitation/hands-up-back",
            "hands-side": "QT/imitation/hands-side-back",
            "Fly": "",
            "hands-up-resume": "QT/imitation/hands-up-back",
            "hands-side-resume": "QT/imitation/hands-side-back",
            "Fly-resume": "",
        }

        # Game performance
        self.game_performance = GamePerformance()
        self.game_id = "imitation"
        self.game_activity_ids = {
            "hands-up": "hands-up",
            "hands-side": "hands-side",
            "Fly": "Fly",
            "hands-up-resume": "hands-up",
            "hands-side-resume": "hands-side",
            "Fly-resume": "Fly",
        }

        self.difficulty_levels = {
            "hands-up": 1,
            "hands-side": 1,
            "Fly": 1,
            "hands-up-resume": 1,
            "hands-side-resume": 1,
            "Fly-resume": 1,
        }

        self.answer_correctnesses = {
            "right": 1,
            "right_after_wrong": 1,
            "wrong": 0,
            "wrong_again": 0,
        }

    def game_performance_callback(self, msg):
        self.game_performance = msg
        rospy.set_param(
            "/migrave/game_performance/participant_id", msg.person.id)

    def game_status_callback(self, msg):
        self.game_status = msg.data
        # start the game
        if self.game_status == "start_imitation":
            self.game_start()

        # start the task
        if self.game_status in self.tasks:
            self.task_start()

        # end the game
        if self.game_status == "end_imitation":
            # Publish game performance when ending
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "game_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)
            # Clear the picture on the tablet
            rospy.loginfo("Game ends")
            rospy.sleep(2)
            rospy.loginfo("Publish image: Nix")
            self.tablet_image_pub.publish("Nix")

    def game_start(self):
        # publish game performance
        self.game_performance.answer_correctness = -1
        self.game_performance.game_activity.game_id = self.game_id
        self.game_performance.game_activity.game_activity_id = "game_init"
        self.game_performance.stamp = rospy.Time.now()
        self.game_performance_pub.publish(self.game_performance)

        rospy.set_param(
            "/migrave/game_performance/game_id", self.game_id)

        # reset counters
        self.count = 0
        self.correct = 0
        # Introduction
        rospy.loginfo("Game starts!")
        rospy.loginfo("Start intro")
        self.migrave_talk_text("Jetzt üben wir Bewegungen nachmachen.")
        self.migrave_talk_text(
            "Das Lerner-Tablet bekommt dehr, oder die, Erwachsene")
        self.migrave_show_emotion("showing_smile")
        self.migrave_talk_text("Los geht's!")
        self.migrave_show_emotion("showing_smile")
        rospy.loginfo("End of intro")

    def task_start(self):
        # Reset the counters when starting a new task
        # Keep the counters' values for a resumed task
        if "resume" not in self.game_status:
            self.count = 0
            self.correct = 0
        # Publish the game performance when resuming
        if "resume" in self.game_status:
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = self.game_status
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        self.task = self.game_status
        self.migrave_talk_text("Hände auf den Tisch, schau mich an.")
        self.migrave_talk_text(
            "Ich mach vor und du schaust zu! Danach bist du dran!")
        self.migrave_talk_text(self.texts[self.task])
        self.migrave_gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.migrave_gesture_play(self.gestures_restore[self.task])
        self.migrave_talk_text("Du bist dran. Mach nach!")
        # Update game status
        self.task_status = "Running"
        self.task_status_pub.publish("Running")
        rospy.loginfo("Publish task status: Running")
        rospy.sleep(1)
        # Show choices (right, Almost right, wrong) on Educator Tablet
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")

    def game_answer_callback(self, msg):
        self.result = msg.data
        rospy.loginfo(f"Game result: {self.result}")
        self.game_grade()

    def game_grade(self):
        # Feedback after grading
        feedback_emotions = {
            "right": "showing_smile",
            "right_after_wrong": "showing_smile",
            "wrong": "",
            "wrong_again": "",
        }
        right_texts = {
            "hands-up": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
            "hands-up-resume": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side-resume": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly-resume": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        right_after_wrong_texts = {
            "hands-up": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
            "hands-up-resume": "Rigchtig, Arme hoch! Wunderbar!",
            "hands-side-resume": "Richtig, Arme zur Seite! Wunderbar!",
            "Fly-resume": "Richtig, Arme seitlich rauf und runter! Wunderbar!",
        }
        feedback_texts = {
            "right": right_texts[self.task],
            "right_after_wrong": right_after_wrong_texts[self.task],
            "wrong": "Schau nochmal genau hin!",
            "wrong_again": "Schau nochmal genau hin!",
        }
        feedback_gestures = {
            "right": "QT/clapping",
            "right_after_wrong": "QT/clapping",
            "wrong": "",
            "wrong_again": "",
        }
        result = self.result

        # Publish game performance when getting the result
        self.game_performance.stamp = rospy.Time.now()
        self.game_performance.game_activity.game_id = self.game_id
        self.game_performance.game_activity.game_activity_id = self.game_activity_ids[
            self.task
        ]
        self.game_performance.game_activity.difficulty_level = self.difficulty_levels[
            self.task
        ]
        self.game_performance.answer_correctness = self.answer_correctnesses[result]
        self.game_performance_pub.publish(self.game_performance)
        rospy.loginfo("Publish game performance")

        if self.count < 5:
            # Reaction after grading

            # self.migrave_gesture_play(self.gestures_restore[self.task])

            emotion = feedback_emotions[result]
            self.migrave_show_emotion(emotion)

            text = feedback_texts[result]
            self.migrave_talk_text(text)

            gesture = feedback_gestures[result]
            self.migrave_gesture_play(gesture)

            if self.result == "right":
                self.count += 1
                self.correct += 1
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")

                # Finish the task when correct >=4 times in the first 5 rounds
                if self.count == 5 and self.correct >= 4:
                    self.migrave_talk_text(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    self.migrave_audio_play("rfh-koeln/MIGRAVE/Reward2")
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)
                    self.tablet_image_pub.publish(image)
                    rospy.sleep(6)
                    rospy.loginfo("Ending")
                    rospy.loginfo(
                        f"Count: {self.count}; Correct: {self.correct}")
                    self.finish_one_task()

                # For other cases, start a new round
                else:
                    rospy.loginfo("Continue")
                    self.migrave_talk_text(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    self.migrave_audio_play("rfh-koeln/MIGRAVE/Reward2")
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)

                    self.tablet_image_pub.publish(image)
                    rospy.loginfo(f"Publish image: {self.correct}Token")
                    rospy.sleep(6)

                    self.migrave_show_emotion("showing_smile")
                    self.migrave_talk_text("Noch ein mal!")

                    self.start_new_round_and_grade()

            if result == "wrong":
                self.count += 1
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")
                self.retry_after_wrong()

            if result == "right_after_wrong":
                self.migrave_show_emotion("showing_smile")
                self.migrave_talk_text("Noch ein mal!")
                self.start_new_round_and_grade()

            if result == "wrong_again":
                self.retry_after_wrong()

        else:  # self.count = 5, self.correct <=4 at the first iteration
            # rospy.sleep(2)
            # self.migrave_gesture_play(self.gestures_restore[self.task])
            emotion = feedback_emotions[result]
            self.migrave_show_emotion(emotion)
            text = feedback_texts[result]
            self.migrave_talk_text(text)
            gesture = feedback_gestures[result]
            self.migrave_gesture_play(gesture)
            if self.count == 5 and self.correct == 4:
                rospy.loginfo("80% correctness case")
                if result == "right":
                    self.count += 1
                    self.correct += 1
                    rospy.loginfo(
                        f"Count: {self.count}; Correct: {self.correct}")
                    self.migrave_talk_text(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    self.migrave_audio_play("rfh-koeln/MIGRAVE/Reward2")
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)

                    self.tablet_image_pub.publish(image)
                    rospy.loginfo(f"Publish image: {self.correct}Token")
                    rospy.sleep(6)
                elif result == "wrong" or result == "wrong_again":
                    self.retry_after_wrong()
                else:  # right_after_wrong
                    self.migrave_talk_text("Noch ein mal!")
                    self.start_new_round_and_grade()
            if self.count == 5 and self.correct <= 3:
                rospy.loginfo("less than 80% correctness case")
                if result == "right":
                    self.correct += 1
                    rospy.loginfo(
                        f"Count: {self.count}; Correct: {self.correct}")
                    self.migrave_talk_text(
                        "Dafür bekommst du einen Stern! Schau mal auf das Tablet."
                    )
                    self.migrave_audio_play("rfh-koeln/MIGRAVE/Reward2")
                    image = f"{self.correct}Token"
                    rospy.loginfo(image)

                    self.tablet_image_pub.publish(image)
                    rospy.loginfo(f"Publish image: {self.correct}Token")
                    rospy.sleep(6)
                    if self.count == 5 and self.correct == 4:
                        rospy.loginfo("3 out of 5 -> 4 correct, finish")
                        self.finish_one_task()
                    if self.count == 5 and self.correct < 4:
                        self.migrave_talk_text("Noch ein mal!")
                        self.start_new_round_and_grade()
                if result == "right_after_wrong":
                    self.migrave_talk_text("Noch ein mal!")
                    self.start_new_round_and_grade()
                if result == "wrong" or result == "wrong_again":
                    self.retry_after_wrong()

            if self.correct == 5:  # finish the task when correct 5 times
                rospy.loginfo("Ending")
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")
                self.finish_one_task()

    def retry_after_wrong(self):
        self.migrave_talk_text(self.texts[self.task])
        self.migrave_gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.migrave_gesture_play(self.gestures_restore[self.task])
        self.migrave_talk_text("Du bist dran. Mach nach!")
        self.task_status_pub.publish("Running")
        rospy.loginfo("Publish task status: Running")
        rospy.sleep(1)
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")
        rospy.loginfo("Wait for grading")

    def start_new_round_and_grade(self):
        self.migrave_talk_text(self.texts[self.task])
        self.migrave_gesture_play(self.gestures[self.task])
        rospy.sleep(2)
        self.migrave_gesture_play(self.gestures_restore[self.task])
        self.migrave_talk_text("Du bist dran. Mach nach!")
        self.task_status_pub.publish("Running")
        rospy.loginfo("Publish task status: Running")
        rospy.sleep(1)
        self.show_educator_choice_pub.publish(True)
        rospy.loginfo("Publish choice")
        rospy.loginfo("Wait for grading")

    def finish_one_task(self):
        self.migrave_show_emotion("showing_smile")
        self.migrave_talk_text("Geschafft! Das hast du super gemacht!")
        self.migrave_gesture_play("QT/Dance/Dance-1-1")
        self.migrave_show_emotion("showing_smile")
        self.migrave_gesture_play("QT/imitation/hands-up-back")
        self.task_status_pub.publish("finish")
        rospy.loginfo("Publish task status: finish")
        self.migrave_talk_text(
            "Schau mal auf das Tablet. Da ist ein Feuerwerk für dich!"
        )
        self.tablet_image_pub.publish("Fireworks")
        rospy.loginfo("Publish image: Fireworks")
        rospy.sleep(1)
        self.migrave_audio_play("rfh-koeln/MIGRAVE/Fireworks")
        # rospy.sleep(6)
        # self.tablet_image_pub.publish("Nix")
        # rospy.loginfo("Publish image: Nix")
        self.count = 0
        self.correct = 0

    def migrave_show_emotion(self, emotion):
        qt_emotion_show = rospy.ServiceProxy(
            "/qt_robot/emotion/show", emotion_show)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/emotion/show")
        try:
            # call the emotion show service
            qt_emotion_show(emotion)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_talk_text(self, text):
        qt_talk_text = rospy.ServiceProxy(
            "/qt_robot/behavior/talkText", behavior_talk_text
        )
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/behavior/talkText")
        try:
            # call the talk text service
            qt_talk_text(text)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_gesture_play(self, gesture):
        qt_gesture_play = rospy.ServiceProxy(
            "/qt_robot/gesture/play", gesture_play)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/gesture/play")
        try:
            # call the gesture play service
            qt_gesture_play(gesture, self.gesture_speed)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_audio_play(self, audio):
        qt_audio_play = rospy.ServiceProxy("/qt_robot/audio/play", audio_play)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/audio/play")
        try:
            # call the audio play service
            qt_audio_play(audio, "")
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
