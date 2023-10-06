#! /usr/bin/python2
# -*- coding: utf-8 -*-
import argparse
import codecs
import sys
from time import sleep

import qi


class TTSCore:
    def __init__(self):
        self.isConnected = False
        self.session = None
        self.speak_move_service = None
        self.tts_service = None
        self.speak_move_service_name = "ALSpeakingMovement"
        self.tts_service_name = "ALAnimatedSpeech"  # "ALTextToSpeech"
        self.tts_body_language_conf = {"bodyLanguageMode": "contextual"}
        # self.language = "Italian"

    def connect(self, ip, port, language):
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + ip + ":" + str(port))
            self.isConnected = True
            self.tts_service = self.session.service(self.tts_service_name)
            # self.tts_service.setLanguage(language)
            print("Connected to Pepper")
        except RuntimeError:
            print(
                "Can't connect to Naoqi at ip \""
                + ip
                + '" on port '
                + str(port)
                + ".\n"
                "Please check your script arguments. Run with -h option for help."
            )
            sys.exit(1)

    def say_from_file(self, text="Ciao Mondo", encoding="utf-8"):
        if self.isConnected:
            assert self.tts_service is not None
            # split the text in sentences
            sentences = text.split(".")
            for sentence in sentences:
                # remove leading and trailing spaces
                sentence = sentence.strip()
                # skip empty sentences
                if sentence == "":
                    continue
                else:
                    # say the sentence
                    self.tts_service.say(sentence, self.tts_body_language_conf)
                    sleep(2)
            # self.tts_service.say(text, self.tts_body_language_conf)

        else:
            print("Not connected to Pepper")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    # IP address of the robot
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Robot IP address")
    # Port number of the robot
    parser.add_argument("--port", type=str, default=9959, help="Robot port number")
    # Language
    parser.add_argument(
        "--language", type=str, default="Italian", help="Robot language"
    )
    # Text to say
    parser.add_argument("--text", type=str, default="Hello, world!", help="Text to say")
    # Encoding
    parser.add_argument("--encoding", type=str, default="utf-8", help="Text encoding")
    # Parse arguments
    args = parser.parse_args()
    # Create a TTSCore object and connect to Pepper
    tts = TTSCore()
    try:
        tts.connect(args.ip, args.port, args.language)
        # Say the text
        tts.say_from_file(args.text, args.encoding)
    except RuntimeError:
        print("I'm unable to speak")
        sys.exit(1)
