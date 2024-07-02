


from aiymakerkit import vision
from aiymakerkit import utils
from pycoral.adapters.detect import BBox
import models
#from detect_objects.py import checkPerson

detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

for frame in vision.get_frames():
    objects = detector.get_objects(frame, threshold=0.4)
    vision.draw_objects(frame, objects, labels)
    
    #if specific classes are detected, print
    for obj in objects:
        label = labels.get(obj.id)
        pos = (obj.bbox.xmin + obj.bbox.xmax)/2
        
        checkPerson(); 
