import sys
sys.path.insert(0, 'aiy-maker-kit/examples')
import detect_objects
from num2words import num2words
from subprocess import call
import time

def speak(text):

        cmd_beg= 'espeak '
        cmd_end= ' | aplay /home/sophie/aiy-maker-kit/examples/Text.wav  2>/dev/null' # To play back the stored .wav file and to dump the std errors to /dev/null
        cmd_out= '--stdout > /home/sophie/aiy-maker-kit/examples/Text.wav ' # To store the voice file


        #Replacing ' ' with '_' to identify words in the text entered
        print(text)
        text = " ".join(text)
        text = text.replace(' ', '_')

        #Calls the Espeak TTS Engine to read aloud a Text
        call([cmd_beg+cmd_out+text+cmd_end], shell=True)

