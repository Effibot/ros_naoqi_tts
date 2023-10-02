#! /usr/bin/python2
# -*- coding: utf-8 -*-
import argparse
import codecs
import sys

import qi


class TTSCore:
    def __init__(self):
        self.isConnected = False
        self.session = None
        self.tts_service = None
        self.tts_service_name = "ALTextToSpeech"
        self.language = "Italian"

    def connect(self, ip, port, language):
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + ip + ":" + str(port))
            self.isConnected = True
            self.tts_service = self.session.service(self.tts_service_name)
            self.tts_service.setLanguage(language)
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
            # check if text is a string or a file
            # if isinstance(text, str):
            #    # then just say it
            #    self.tts_service.say(text)

            # else:
            # read the file with appropriate encoding and say it
            with codecs.open(text, "r", encoding) as f:
                content = f.read()
                to_say = content.encode(encoding)
            self.tts_service.say(to_say)
        else:
            print("Not connected to Pepper")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    # IP address of the robot
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Robot IP address")
    # Port number of the robot
    parser.add_argument("--port", type=str, default="9559", help="Robot port number")
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
