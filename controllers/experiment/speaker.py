from controller import Speaker


class Speech():
    """For this to work the 'Robot "NAO"' in the object panel in webots needs to have
    the base object speaker as a child. To be able to add children Nao must be transformed
    into a base/root (?) node.
    
    https://cyberbotics.com/doc/reference/speaker?tab-language=python
    """
    # load the preprogrammed sentences from settings
    file = open("settings.json", "r")
    settings = file.read()
    self.settings = eval(settings)["speech"]   # eval converts it into a dict
    file.close()
    
    # init the webots Speaker object
    self.speaker = Speaker("speaker")
    
    print(speaker.getEngine())
    print(speaker.getLanguage())
    
    
    def speak(self, txt_key, volume=1):
        # text = 'Hello! Using the text-to-speech of the Speaker device, I can speak 6 different languages: English with US or UK accent, German, Spanish, French and Italian. Using tags I can modulate my speech, like for example change <prosody pitch="+16.8st">the pitch of my voice</prosody>, <prosody pitch="-15st">and speak with a very low pitch</prosody>. <prosody rate="0.5">And I can change the speed</prosody><prosody rate="1.5">at which I speak</prosody>. I can also <prosody volume="20">adjust the volume of my voice</prosody>.'
        txt = None
        if txt_key == "intro":
            txt = self.sentences[txt_key]
            self.speaker.speak(txt, volume)    
        else:
            txt = self.sentences[txt_key]["text"]
            self.speaker.speak(txt, volume) 

    def is_speaking(self):
        return self.speaker.is_speaking()




