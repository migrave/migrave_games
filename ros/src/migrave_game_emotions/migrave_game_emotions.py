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
import random


class MigraveGameEmotions:
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
        self.task_status_pub = rospy.Publisher(
            "/migrave_game_emotions/task_status", String, queue_size=10
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
        self.game_status = "Waiting"
        self.task_status = "Waiting"
        self.result = "Waiting"
        self.task = "Waiting"
        self.emotion = "Waiting"
        self.emotion_image = "Nix"

        self.tasks = [
            "happy",
            "sad",
            "happy_vs_sad",
            "sad_vs_happy",
            "happy_or_sad",
            "happy_resume",
            "sad_resume",
            "happy_vs_sad_resume",
            "sad_vs_happy_resume",
            "happy_or_sad_resume",
        ]
        self.images_happy = ["Happy1", "Happy2"]
        self.images_sad = ["Sad1", "Sad2"]

        # Game performance
        self.game_performance = GamePerformance()
        self.game_id = "emotion"
        self.game_activity_ids = {
            "happy": "happy",
            "sad": "sad",
            "happy_vs_sad": "happy_vs_sad",
            "sad_vs_happy": "sad_vs_happy",
            "happy_or_sad": "happy_or_sad",
            "happy_resume": "happy",
            "sad_resume": "sad",
            "happy_vs_sad_resume": "happy_vs_sad",
            "sad_vs_happy_resume": "sad_vs_happy",
            "happy_or_sad_resume": "happy_or_sad",
        }

        self.difficulty_levels = {
            "happy": 1,
            "sad": 1,
            "happy_vs_sad": 2,
            "sad_vs_happy": 2,
            "happy_or_sad": 3,
            "happy_resume": 1,
            "sad_resume": 1,
            "happy_vs_sad_resume": 2,
            "sad_vs_happy_resume": 2,
            "happy_or_sad_resume": 3,
        }

        self.answer_correctnesses = {
            "right": 1,
            "right_1": 1,
            "right_2": 1,
            "wrong": 0,
            "wrong_1": 0,
            "wrong_2": 0,
        }

    def game_performance_callback(self, msg):
        self.game_performance = msg
        rospy.set_param(
            "/migrave/game_performance/participant_id", msg.person.id)

    def game_status_callback(self, msg):
        self.game_status = msg.data
        # start the game
        if self.game_status == "start_emotions":
            self.game_start()
        # start the task
        if self.game_status in self.tasks:
            self.task_start()
        # end the game
        if self.game_status == "end_emotions":
            rospy.loginfo("Game ends")
            # rospy.sleep(6)
            # rospy.loginfo("Publish image: Nix")
            # self.tablet_image_pub.publish("Nix")
            # publish game performance
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "game_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

    def game_answer_callback(self, msg):
        self.result = msg.data
        rospy.loginfo(f"Game result: {self.result}")
        self.game_grade()

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
        rospy.loginfo("Emotion game starts!")
        rospy.loginfo("Start intro")
        self.migrave_talk_text("Jetzt üben wir Gefühle erkennen.")
        self.migrave_talk_text("Los geht's!")
        self.migrave_show_emotion("showing_smile")
        self.migrave_talk_text(
            "Hände auf den Tisch, schau mich an. Los geht's!")
        self.migrave_show_emotion("showing_smile")
        self.migrave_talk_text(
            "Ich nenne dir ein Gefühl. Du tippst auf das passende Bild."
        )
        rospy.loginfo("End of intro")

    def task_start(self):
        rospy.loginfo("Start new task")
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
        rospy.loginfo(f"Task: {self.task}")

        # Update game status
        rospy.sleep(2)
        self.task_status = "running"
        self.task_status_pub.publish("running")
        rospy.loginfo("Publish task status: running")
        rospy.sleep(2)

        # Show image
        if self.task in ["happy", "sad", "happy_resume", "sad_resume"]:
            rospy.loginfo("Simple task")
            self.start_new_round_simple()
        if self.task in [
            "happy_vs_sad",
            "sad_vs_happy",
            "happy_or_sad",
            "happy_vs_sad_resume",
            "sad_vs_happy_resume",
            "happy_or_sad_resume",
        ]:
            rospy.loginfo("Normal task")
            self.start_new_round()

    def game_grade(self):
        result = self.result
        # Feedback after grading
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
        feedback_gestures = {
            "right": "QT/clapping",
            "right_1": "QT/clapping",
            "right_2": "QT/clapping",
            "wrong": "",
            "wrong_1": "",
            "wrong_2": "",
        }

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

                # Finish the task when correct >= 4 times in the first 5 rounds
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

            if result in ["right_1", "right_2"]:
                self.migrave_show_emotion("showing_smile")
                self.migrave_talk_text("Noch ein mal!")
                self.start_new_round_and_grade()
            if result in ["wrong_1", "wrong_2"]:
                self.retry_after_wrong()

        else:  # self.count = 5, self.correct <=4 at the first iteration
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
                elif result in ["wrong", "wrong_1", "wrong_2"]:
                    # self.migrave_show_emotion("showing_smile")
                    # self.migrave_talk_text("Noch ein mal!")
                    self.retry_after_wrong()
                else:  # right_after_wrong
                    self.migrave_talk_text("Noch ein mal")
                    self.start_new_round_and_grade()
            if self.count == 5 and self.correct <= 3:
                rospy.loginfo("less than 80% correctness case")
                if result == "right":
                    self.correct += 1
                    rospy.loginfo(
                        f"Count: {self.count}; Correct {self.correct}")
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
                        rospy.loginfo("Less than 4, continue")
                        self.migrave_talk_text("Noch ein mal!")
                        self.start_new_round_and_grade()
                if result in ["right_1", "right_2"]:
                    self.migrave_talk_text("Noch ein mal!")
                    self.start_new_round_and_grade()
                if result in ["wrong", "wrong_1", "wrong_2"]:
                    self.retry_after_wrong()

            if self.correct == 5:
                rospy.loginfo("Ending")
                rospy.loginfo(f"Count: {self.count}; Correct: {self.correct}")
                self.finish_one_task()

    def retry_after_wrong(self):
        self.task_status_pub.publish("running")
        rospy.loginfo("Publish task status: running")
        rospy.sleep(2)
        # self.migrave_talk_text("Schau auf das Tablet!")

    def start_new_round_and_grade(self):
        self.task_status_pub.publish("running")
        rospy.loginfo("Publish task status: running")
        rospy.sleep(2)

        if self.task in ["happy", "sad", "happy_resume", "sad_resume"]:
            self.start_new_round_simple()

        if self.task in [
            "happy_vs_sad",
            "sad_vs_happy",
            "happy_or_sad",
            "happy_vs_sad_resume",
            "sad_vs_happy_resume",
            "happy_or_sad_resume",
        ]:
            self.start_new_round()

    def start_new_round_simple(self):

        if self.task in ["happy", "happy_resume"]:
            self.emotion = "glückliche"
            self.emotion_image = random.choice(self.images_happy)
        if self.task in ["sad", "sad_resume"]:
            self.emotion = "traurige"
            self.emotion_image = random.choice(self.images_sad)

        self.emotion_pub.publish(self.emotion)
        rospy.loginfo(f"Publish emotion: {self.emotion}")
        rospy.sleep(2)

        rospy.loginfo(f"Show image: {self.emotion_image}")
        self.migrave_talk_text("Schau auf das Tablet!")
        # rospy.sleep(2)
        self.tablet_image_pub.publish(self.emotion_image)
        rospy.sleep(2)

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

        self.emotion_pub.publish(self.emotion)
        # rospy.sleep(1)
        self.migrave_talk_text("Schau auf das Tablet!")
        rospy.sleep(1)

        self.tablet_image_pub.publish(self.images[0])
        rospy.sleep(1)
        self.tablet_image_pub.publish(self.images[1])
        rospy.sleep(1)
        self.tablet_image_pub.publish(self.image_x)
        rospy.sleep(1)
        self.correct_image_pub.publish(str(self.correct_image))
        rospy.sleep(1)

    def finish_one_task(self):
        self.migrave_show_emotion("showing_smile")
        self.migrave_talk_text("Geschafft! Das hast du super gemacht!")
        self.migrave_gesture_play("QT/Dance/Dance-1-1")
        self.migrave_show_emotion("showing_smile")
        self.migrave_gesture_play("QT/imitation/hands-up-back")
        self.task_status_pub.publish("finish")
        rospy.loginfo("Publish task status: finish")
        # rospy.sleep(4)
        self.migrave_talk_text(
            "Schau mal auf das Tablet. Da ist ein Feuerwerk für dich!"
        )
        self.tablet_image_pub.publish("Fireworks")
        rospy.loginfo("Publish image: Fireworks")
        rospy.sleep(2)
        self.migrave_audio_play("rfh-koeln/MIGRAVE/Fireworks")
        # rospy.sleep(10)
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
            # publish robot action info (start showing emotion)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_emotion_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the emotion show service
            qt_emotion_show(emotion)

            # publish robot action info (finish showing emotion)
            self.game_performance.game_activity.game_activity_id = "robot_emotion_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_talk_text(self, text):
        qt_talk_text = rospy.ServiceProxy(
            "/qt_robot/behavior/talkText", behavior_talk_text
        )
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/behavior/talkText")
        try:
            # publish robot action info (start talking)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_talk_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the talk text service
            qt_talk_text(text)

            # publish robot action info (finish talking)
            self.game_performance.game_activity.game_activity_id = "robot_talk_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_gesture_play(self, gesture):
        qt_gesture_play = rospy.ServiceProxy(
            "/qt_robot/gesture/play", gesture_play)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/gesture/play")
        try:
            # publish robot action info (start moving)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_gesture_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the gesture play service
            qt_gesture_play(gesture, self.gesture_speed)

            # publish robot action info (finish moving)
            self.game_performance.game_activity.game_activity_id = "robot_gesture_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def migrave_audio_play(self, audio):
        qt_audio_play = rospy.ServiceProxy("/qt_robot/audio/play", audio_play)
        # block/wait for ros service
        rospy.wait_for_service("/qt_robot/audio/play")
        try:
            # publish robot action info (start playing audio)
            self.game_performance.answer_correctness = -1
            self.game_performance.game_activity.game_id = self.game_id
            self.game_performance.game_activity.game_activity_id = "robot_audio_start"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

            # call the audio play service
            qt_audio_play(audio, "")

            # publish robot action info (finish audio)
            self.game_performance.game_activity.game_activity_id = "robot_audio_end"
            self.game_performance.stamp = rospy.Time.now()
            self.game_performance_pub.publish(self.game_performance)

        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
